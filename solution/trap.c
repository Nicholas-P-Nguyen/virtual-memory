#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"
#include "wmap.h"
#include "types.h"
#include "sleeplock.h"
#include "fs.h"
#include "file.h" 

extern int mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm);
extern pte_t *walkpgdir(pde_t *pgdir, const void *va, int alloc);


// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[];  // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE<<3, vectors[i], 0);
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE<<3, vectors[T_SYSCALL], DPL_USER);

  initlock(&tickslock, "time");
}

void
idtinit(void)
{
  lidt(idt, sizeof(idt));
}

//PAGEBREAK: 41
void
trap(struct trapframe *tf)
{
  if(tf->trapno == T_SYSCALL){
    if(myproc()->killed)
      exit();
    myproc()->tf = tf;
    syscall();
    if(myproc()->killed)
      exit();
    return;
  }

  switch(tf->trapno){
  case T_IRQ0 + IRQ_TIMER:
    if(cpuid() == 0){
      acquire(&tickslock);
      ticks++;
      wakeup(&ticks);
      release(&tickslock);
    }
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE:
    ideintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE+1:
    // Bochs generates spurious IDE1 interrupts.
    break;
  case T_IRQ0 + IRQ_KBD:
    kbdintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_COM1:
    uartintr();
    lapiceoi();
    break;
  case T_IRQ0 + 7:
  case T_IRQ0 + IRQ_SPURIOUS:
    cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpuid(), tf->cs, tf->eip);
    lapiceoi();
    break;

  case T_PGFLT: 
  {
    uint pageflt_addr = rcr2();
    struct proc *p = myproc();

    if (pageflt_addr >= KERNBASE) {
      cprintf("Segmentation Fault\n");
      kill(p->pid); 
      return;
    }
    // looping through wmap entries to locate page fault addr.
    for (int i = 0; i < p->wmap_count; i++) {
      struct wmap_entry *wmap = &p->wmaps[i];
      uint base = wmap->addr;
      uint bound = wmap->addr + wmap->length;

      if (pageflt_addr >= base && pageflt_addr < bound) {
        
        // Allocate one 4096-byte page of physical memory & setting to 0.
        char *page_frame = kalloc();
        memset(page_frame, 0, PGSIZE);
        if (page_frame == 0) {
          cprintf("Memory couldn't be allocated");
          return;
        }
        // For file-backed, reading from file and storing it into allocated page frame
        if (!(wmap->flags & MAP_ANONYMOUS)) {
          int file_offset = pageflt_addr - wmap->addr;
          int old_off = wmap->f->off;
          // Setting off to the file offset
          wmap->f->off = file_offset;
          fileread(wmap->f, page_frame, PGSIZE);
          wmap->f->off = old_off;
        }
        // mapping vaddr -> page_frame
        int success = mappages(p->pgdir, (void *)pageflt_addr, PGSIZE, V2P(page_frame), PTE_W | PTE_U);
        if (success < 0) {
          cprintf("Page mapping failed\n");
          kfree(page_frame);
          return;
        }
        wmap->num_pages_loaded++;

        return;
      }
    }
    cprintf("Segmentation Fault\n");
    kill(p->pid);
  }
  //PAGEBREAK: 13
  default:
    if(myproc() == 0 || (tf->cs&3) == 0){
      // In kernel, it must be our mistake.
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpuid(), tf->eip, rcr2());
      panic("trap");
    }
    // In user space, assume process misbehaved.
    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            myproc()->pid, myproc()->name, tf->trapno,
            tf->err, cpuid(), tf->eip, rcr2());
    myproc()->killed = 1;
  }

  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, let it keep running
  // until it gets to the regular system call return.)
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();

  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  if(myproc() && myproc()->state == RUNNING &&
     tf->trapno == T_IRQ0+IRQ_TIMER)
    yield();

  // Check if the process has been killed since we yielded
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();
}


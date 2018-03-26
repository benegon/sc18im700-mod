/*
 * sc18im700.c - SC18IM700 I2C adaptor driver
 *
 * This file is derived from linux/drivers/net/can/slcan.c
 * and linux/dirvers/i2c/bcm2835.c
 * as well as this patch https://patchwork.kernel.org/patch/4224461/

 * sc18im700.c Author  : James Bryant <james@uberfoo.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/tty.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/limits.h>

// Arbitrary line discipline number
#define N_SC18IM700   29

MODULE_ALIAS_LDISC(N_SC18IM700);
MODULE_DESCRIPTION("SC18IM700 serial line I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("James Bryant <james@uberfoo.net>");

#define OVERHEAD_BYTES 4

static int max_message_len = 1024 - OVERHEAD_BYTES;
module_param(max_message_len, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(max_message_len, "The maximum length of an I2C message.");

static int debug_level = 0;
module_param(debug_level, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(debug_level, "The verbosity level of debug messaging.");

// Arbitrary magic number
#define SC18IM700_MAGIC      0x53CB

#define SC18IM700_START      0x53
#define SC18IM700_STOP       0x50
#define SC18IM700_READ_REG   0x52
#define SC18IM700_WRITE_REG  0x57
#define SC18IM700_READ_GPIO  0x49
#define SC18IM700_WRITE_GPIO 0x4F
#define SC18IM700_POWER_DOWN 0x5A

#define SC18IM700_STAT_REG   0x0A

#define SC18IM700_OK           0x00
#define SC18IM700_NACK_ON_ADDR 0x01
#define SC18IM700_NACK_ON_DATA 0x02
#define SC18IM700_TIMEOUT      0x08
#define DEB1(x) if (debug_level >= 1) x

struct sc18im700_dev {
  int                    magic;
  struct tty_struct      *tty;                    /* ptr to TTY structure     */
  struct i2c_adapter     adapter;                 /* ptr to I2C structure     */
  struct work_struct     tx_work;                 /* Flushes transmit buffer  */
  unsigned char          *rbuff;                  /* receiver buffer          */
  int                    rcount;                  /* received chars counter   */
  unsigned char          *xbuff;                  /* transmitter buffer	      */
  unsigned char          *xhead;                  /* pointer to next XMIT byte*/
  int                    xleft;                   /* bytes left in XMIT queue */
  spinlock_t             lock;                    /* xmit lock                */
  struct completion      xcompletion;             /* read completion          */
  struct completion      rcompletion;             /* xmit completion          */
  struct i2c_msg         *curr_msg;               /* the current message      */
};

static const struct i2c_adapter_quirks sc18im700_i2c_quirks = {
  .flags = 0,
};

static int sc18im700_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
      int num)
{
  struct sc18im700_dev *sl = i2c_get_adapdata(adap);

  struct i2c_msg *msg;
  int curmsg, addr, numbytes, actual, i;
  unsigned char *pos;
  unsigned long time_left;

  DEB1(printk(KERN_DEBUG "sc18im700: Transfer Starts"));

  curmsg = 0;
  while (curmsg < num) {
    numbytes = 0;
    msg = &msgs[curmsg];

    DEB1(printk(KERN_DEBUG "sc18im700: Operation: %s, "
      "Addr: 0x%02x, Length: %d, "
      "Current Transfer No: %d, "
      "Total No of transfer: %d, "
      "Flags: 0x%02x",
      (msg->flags & I2C_M_RD) ? "Read" : "Write",
      msg->addr, msg->len, (curmsg + 1), num, msg->flags));

    if (msg->len > max_message_len) {
      printk(KERN_ERR "sc18im700: message length %d is greater than allowed (%d)\n", msg->len, max_message_len);
      return -EIO;
    }

    // Address is in the 7 most significant bits
    addr = (msg->addr & 0x7F) << 1;

    // Least significant bit is R/W flag
    // Check if this is a read operation
    if (msg->flags & I2C_M_RD)
      addr |= 1;
    if (msg->flags & I2C_M_REV_DIR_ADDR)
      addr ^= 1;

    // Write out the beginning of the message
    pos = sl->xbuff;
    *pos++ = SC18IM700_START;
    *pos++ = addr;
    *pos++ = msg->len;

    // If this is a write, write out the message body
    if (!(msg->flags & I2C_M_RD)) {
      for (i = 0; i < msg->len; i++) {
        *pos++ = msg->buf[i];
      }
    }

    // Write out the stop marker
    *pos++ = SC18IM700_STOP;

    if (!(msg->flags & I2C_M_RD)) {
      *pos++ = SC18IM700_READ_REG;
      *pos++ = SC18IM700_STAT_REG;
      *pos++ = SC18IM700_STOP;
    }

    // The ordering here is important
    reinit_completion(&sl->rcompletion);
    reinit_completion(&sl->xcompletion);
    set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
    actual = sl->tty->ops->write(sl->tty, sl->xbuff, pos - sl->xbuff);
    sl->xleft = (pos - sl->xbuff) - actual;
    sl->xhead = sl->xbuff + actual;

    // If we didn't xmit the whole message, we need to wait for it to finish
    if (sl->xleft > 0) {
      time_left = wait_for_completion_timeout(&sl->xcompletion, adap->timeout);
      if (!time_left) {
        printk(KERN_ERR "sc18im700: i2c write timed out");
        return -ETIMEDOUT;
      }
    }

    time_left = wait_for_completion_timeout(&sl->rcompletion, adap->timeout);
    if (!time_left) {
      printk(KERN_ERR "sc18im700: i2c read timed out");
      return -ETIMEDOUT;
    }

    if (msg->flags & I2C_M_RD) {
      // If this is a read, drain read buffer into message buffer
      for (i = 0; i < sl->rcount; i++) {
        msg->buf[i] = sl->rbuff[i];
      }
    } else {
      // If this is a write, check the return from the status register
      sl->rcount = 0; // Reset the read counter
      if (sl->rbuff[0] != SC18IM700_OK) {
        if (sl->rbuff[0] & SC18IM700_NACK_ON_ADDR) {
          pr_info("sc18im700: i2c NACK on address");
          return -EREMOTEIO;
        }
        if (sl->rbuff[0] & SC18IM700_NACK_ON_DATA) {
          printk(KERN_ERR "sc18im700: i2c NACK on data");
          return -EREMOTEIO;
        }
        printk(KERN_ERR "sc18im700: i2c timed out");
        return -ETIMEDOUT;
      }
    }
    sl->rcount = 0; // Reeset the read counter

    curmsg++;
  }

  return num;
}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void sc18im700_transmit(struct work_struct *work)
{
  struct sc18im700_dev *sl = container_of(work, struct sc18im700_dev, tx_work);
  int actual;

  spin_lock_bh(&sl->lock);
  /* First make sure we're connected. */
  if (!sl->tty || sl->magic != SC18IM700_MAGIC) {
    spin_unlock_bh(&sl->lock);
    return;
  }

  if (sl->xleft <= 0)  {
    /* Now serial buffer is almost free & we can start
     * transmission of another packet */
    clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
    complete(&sl->xcompletion);
    spin_unlock_bh(&sl->lock);
    return;
  }

  actual = sl->tty->ops->write(sl->tty, sl->xhead, sl->xleft);
  sl->xleft -= actual;
  sl->xhead += actual;
  spin_unlock_bh(&sl->lock);
}

static u32 sc18im700_i2c_func(struct i2c_adapter *adap)
{
  return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sc18im700_i2c_algo = {
  .master_xfer	  = sc18im700_i2c_xfer,
  .functionality	= sc18im700_i2c_func,
};

static int sc18im700_open(struct tty_struct *tty)
{
  struct sc18im700_dev *sl;
  int err;
  struct i2c_adapter *adap;
  //struct termios tattr;

  tty->termios.c_lflag &= ~(ICANON | ECHO);

  if (tty->ops->write == NULL)
    return -EOPNOTSUPP;

  if (tty->ops->set_termios == NULL)
    return -EOPNOTSUPP;

  tty->ops->set_termios(tty, &tty->termios);

  sl = tty->disc_data;

  err = -EEXIST;
  /* First make sure we're not already connected. */
  if (sl && sl->magic == SC18IM700_MAGIC)
    return err;

  sl = kzalloc(sizeof(*sl), GFP_KERNEL);

  sl->magic = SC18IM700_MAGIC;
  sl->tty = tty;

  sl->xbuff = kzalloc(max_message_len + OVERHEAD_BYTES, GFP_KERNEL);
  sl->rbuff = kzalloc(max_message_len, GFP_KERNEL);

  init_completion(&sl->xcompletion);
  init_completion(&sl->rcompletion);

  spin_lock_init(&sl->lock);
  INIT_WORK(&sl->tx_work, sc18im700_transmit);

  tty->disc_data = sl;

  tty->receive_room = 65536;	/* We don't flow control */

  adap = &sl->adapter;
  i2c_set_adapdata(adap, sl);
  adap->owner = THIS_MODULE;
  adap->class = I2C_CLASS_DEPRECATED;
  strlcpy(adap->name, "sc18im700 I2C adapter", sizeof(adap->name));
  adap->algo = &sc18im700_i2c_algo;
  adap->dev.parent = tty->dev;
  adap->dev.of_node = tty->dev->of_node;
  adap->quirks = &sc18im700_i2c_quirks;

  return i2c_add_adapter(adap);
}

static void sc18im700_close(struct tty_struct *tty)
{
  struct sc18im700_dev *sl = (struct sc18im700_dev *) tty->disc_data;

  /* First make sure we're connected. */
  if (!sl || sl->magic != SC18IM700_MAGIC || sl->tty != tty)
    return;

  flush_work(&sl->tx_work);

  spin_lock_bh(&sl->lock);
  tty->disc_data = NULL;
  sl->tty = NULL;
  spin_unlock_bh(&sl->lock);

  i2c_del_adapter(&sl->adapter);

  kfree(sl->xbuff);
  kfree(sl->rbuff);
  kfree(sl);
}

static int sc18im700_hangup(struct tty_struct *tty)
{
  sc18im700_close(tty);
  return 0;
}

/* Perform I/O control on an active SLCAN channel. */
static int sc18im700_ioctl(struct tty_struct *tty, struct file *file,
         unsigned int cmd, unsigned long arg)
{
  // struct sc18im700_dev *sl = (struct sc18im700_dev *) tty->disc_data;
  return tty_mode_ioctl(tty, file, cmd, arg);
}

/*
 * Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void sc18im700_write_wakeup(struct tty_struct *tty)
{
  struct sc18im700_dev *sl = (struct sc18im700_dev *) tty->disc_data;

  schedule_work(&sl->tx_work);
}

/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of data has been received. This will not be re-entered
 * while running but other ldisc functions may be called
 * in parallel
 */

static void sc18im700_receive_buf(struct tty_struct *tty,
        const unsigned char *cp, char *fp, int count)
{
  struct sc18im700_dev *sl = (struct sc18im700_dev *) tty->disc_data;

  if (!sl || sl->magic != SC18IM700_MAGIC)
    return;

  /* Read the characters out of the buffer */
  while (count--) {
    if (fp && *fp++) {
      cp++;
      continue;
    }
    sl->rbuff[sl->rcount++] = *cp++;
  }
  if (sl->rcount >= sl->curr_msg->len) {
    complete(&sl->rcompletion);
  }
}


static struct tty_ldisc_ops sc18im700_ldisc = {
  .owner		= THIS_MODULE,
  .magic		= TTY_LDISC_MAGIC,
  .name		  = "sc18im700",
  .open		  = sc18im700_open,
  .close		= sc18im700_close,
  .hangup		= sc18im700_hangup,
  .ioctl		= sc18im700_ioctl,
  .receive_buf	= sc18im700_receive_buf,
  .write_wakeup	= sc18im700_write_wakeup,
};

static int __init sc18im700_init(void)
{
  int status;

  pr_info("sc18im700: SC18IM700 serial line I2C bus adapter\n");

  /* Fill in our line protocol discipline, and register it */
  status = tty_register_ldisc(N_SC18IM700, &sc18im700_ldisc);
  if (status)  {
    printk(KERN_ERR "sc18im700: can't register line discipline: %d\n", status);
  } else {
  	printk(KERN_DEBUG "sc18im700: registered\n");
  }
  return status;
}

static void __exit sc18im700_exit(void)
{
  /* *** TODO *** First of all: check for active disciplines and hangup them.
   */

  int i = tty_unregister_ldisc(N_SC18IM700);
  if (i)
    printk(KERN_ERR "sc18im700: can't unregister line discipline (err %d)\n", i);

}

module_init(sc18im700_init);
module_exit(sc18im700_exit);

/*
 * File      : stm32f4xx_hcd.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-05-16     Yi Qiu       first version
 * 2012-12-05     heyuanjie87  add interrupt transfer
 * 2019-01-28     weety        Support new usb stack
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include "usb_core.h"
#include "usb_hcd_int.h"
#include "usbh_ioreq.h"
#include "usbh_hcs.h"
#include "usb_bsp.h"

#define OTG_FS_PORT 1
static struct uhcd stm32_hcd;
static struct rt_completion urb_completion;
static USBH_HOST USB_Host;
ALIGN(4) static USB_OTG_CORE_HANDLE USB_OTG_Core;

void OTG_FS_IRQHandler(void)
{
    rt_interrupt_enter();
    USBH_OTG_ISR_Handler(&USB_OTG_Core);
    rt_interrupt_leave();
}

/**
  * @brief  USBH_DeInit
  *         Re-Initialize Host
  * @param  None
  * @retval status: USBH_Status
  */
USBH_Status USBH_DeInit(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
    /* Software Init */

    phost->gState = HOST_IDLE;
    phost->gStateBkp = HOST_IDLE;
    phost->EnumState = ENUM_IDLE;
    phost->RequestState = CMD_SEND;

    phost->Control.state = CTRL_SETUP;
    phost->Control.ep0size = USB_OTG_MAX_EP0_SIZE;

    phost->device_prop.address = USBH_DEVICE_ADDRESS_DEFAULT;
    phost->device_prop.speed = HPRT0_PRTSPD_FULL_SPEED;

    USBH_Free_Channel  (pdev, phost->Control.hc_num_in);
    USBH_Free_Channel  (pdev, phost->Control.hc_num_out);

    return USBH_OK;
}

static __IO rt_bool_t connect_status = RT_FALSE;

/**
  * @brief  USBH_Connect
  *         USB Connect callback function from the Interrupt.
  * @param  selected device
  * @retval none
  */
rt_uint8_t usbh_connect (USB_OTG_CORE_HANDLE *pdev)
{
    rt_kprintf("usbh_connect\n");

    uhcd_t hcd = &stm32_hcd;
    pdev->host.ConnSts = 1;
    if (!connect_status)
    {
        connect_status = RT_TRUE;
        RT_DEBUG_LOG(RT_DEBUG_USB, ("connected\n"));
        rt_usbh_root_hub_connect_handler(hcd, OTG_FS_PORT, RT_FALSE);
    }

    return 0;
}

/**
  * @brief  USBH_Disconnect
  *         USB Disconnect callback function from the Interrupt.
  * @param  selected device
  * @retval none
  */
rt_uint8_t usbh_disconnect (USB_OTG_CORE_HANDLE *pdev)
{
    rt_kprintf("usbh_disconnect\n");
    uhcd_t hcd = &stm32_hcd;
    pdev->host.ConnSts = 0;
    if (connect_status)
    {
        connect_status = RT_FALSE;
        RT_DEBUG_LOG(RT_DEBUG_USB, ("disconnnect\n"));
        rt_usbh_root_hub_disconnect_handler(hcd, OTG_FS_PORT);
    }

    return 0;
}

rt_uint8_t usbh_sof (USB_OTG_CORE_HANDLE *pdev)
{
#if 0
    /* This callback could be used to implement a scheduler process */
    uint16_t i;
    static uint16_t sofcnt = 0;
    uep_desc_t ep;
    
    sofcnt ++;
    for (i = 2; i < MAX_HC; i ++)
    {
        if ((_xfer[i].flag & PXFER_FLAG_READY) && (_xfer[i].pipe != RT_NULL))
        {
            ep = &_xfer[i].pipe->ep;
            if ((sofcnt > _xfer[i].fnum))
            {
                if ((_xfer[i].buffer != RT_NULL) && (_xfer[i].size != 0))
                {
                    if (ep->bEndpointAddress & USB_DIR_IN)
                        USBH_InterruptReceiveData(&USB_OTG_Core, _xfer[i].buffer, _xfer[i].size, i);
                    else
                        USBH_InterruptSendData(&USB_OTG_Core, _xfer[i].buffer, _xfer[i].size, i);
                    _xfer[i].fnum = sofcnt + ep->bInterval + 30;
                }
            }
        }
    }
#endif
    return 0;
}

void usbh_urb_done (USB_OTG_CORE_HANDLE *pdev, uint32_t hc)
{
    RT_DEBUG_LOG(RT_DEBUG_USB, ("urbdone\n"));
    rt_completion_done(&urb_completion);
}

void usbh_urb_nak (USB_OTG_CORE_HANDLE *pdev, uint8_t hc)
{
    if (pdev->host.hc[hc].ep_type == EP_TYPE_BULK && !pdev->host.hc[hc].ep_is_in)
    {
        rt_completion_done(&urb_completion);
    }
}

static USBH_HCD_INT_cb_TypeDef USBH_HCD_INT_cb =
{
    usbh_sof,
    usbh_connect,
    usbh_disconnect,
    usbh_urb_done,
    usbh_urb_nak,
};

USBH_HCD_INT_cb_TypeDef  *USBH_HCD_INT_fops = &USBH_HCD_INT_cb;

USBH_Status USBH_HC_SubmitRequest(USB_OTG_CORE_HANDLE *pdev,
                                           uint8_t ch_num, 
                                           uint8_t direction,
                                           uint8_t ep_type,  
                                           uint8_t token, 
                                           uint8_t* pbuff, 
                                           uint16_t length,
                                           uint8_t do_ping) 
{
    pdev->host.hc[ch_num].ep_is_in = direction;
    pdev->host.hc[ch_num].ep_type  = ep_type;

  if(token == 0)
  {
    pdev->host.hc[ch_num].data_pid = HC_PID_SETUP;
  }
  else
  {
    pdev->host.hc[ch_num].data_pid = HC_PID_DATA1;
  }
  
  /* Manage Data Toggle */
  switch(ep_type)
  {
  case EP_TYPE_CTRL:
    if((token == 1) && (direction == 0)) /*send data */
    {
      if (length == 0)
      { /* For Status OUT stage, Length==0, Status Out PID = 1 */
        pdev->host.hc[ch_num].toggle_out = 1;
      }
      
      /* Set the Data Toggle bit as per the Flag */
      if (pdev->host.hc[ch_num].toggle_out == 0)
      { /* Put the PID 0 */
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA0;    
      }
      else
      { /* Put the PID 1 */
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA1;
      }
      if(pdev->host.URB_State[ch_num]  != URB_NOTREADY)
      {
        pdev->host.hc[ch_num].do_ping = do_ping;
      }
    }
    break;
  
  case EP_TYPE_BULK:
    if(direction == 0)
    {
      /* Set the Data Toggle bit as per the Flag */
      if ( pdev->host.hc[ch_num].toggle_out == 0)
      { /* Put the PID 0 */
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA0;    
      }
      else
      { /* Put the PID 1 */
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA1;
      }
      if(pdev->host.URB_State[ch_num]  != URB_NOTREADY)
      {
        pdev->host.hc[ch_num].do_ping = do_ping;
      }
    }
    else
    {
      if( pdev->host.hc[ch_num].toggle_in == 0)
      {
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA0;
      }
      else
      {
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA1;
      }
    }
    
    break;
  case EP_TYPE_INTR:
    if(direction == 0)
    {
      /* Set the Data Toggle bit as per the Flag */
      if ( pdev->host.hc[ch_num].toggle_out == 0)
      { /* Put the PID 0 */
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA0;    
      }
      else
      { /* Put the PID 1 */
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA1;
      }
    }
    else
    {
      if( pdev->host.hc[ch_num].toggle_in == 0)
      {
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA0;
      }
      else
      {
        pdev->host.hc[ch_num].data_pid = HC_PID_DATA1;
      }
    }
    break;
    
  case EP_TYPE_ISOC: 
    pdev->host.hc[ch_num].data_pid = HC_PID_DATA0;
    break;  
  }
  
  pdev->host.hc[ch_num].xfer_buff = pbuff;
  pdev->host.hc[ch_num].xfer_len  = length;
  pdev->host.URB_State[ch_num] = URB_IDLE;  
  pdev->host.hc[ch_num].xfer_count = 0;
  pdev->host.channel[ch_num] = ch_num;
  pdev->host.HC_Status[ch_num] = HC_IDLE;
  
  return HCD_SubmitRequest (pdev , ch_num);
}

/**
  * @brief  Return the Host Channel state.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval Host channel state
  *          This parameter can be one of these values:
  *            HC_IDLE/
  *            HC_XFRC/
  *            HC_HALTED/
  *            HC_NYET/ 
  *            HC_NAK/  
  *            HC_STALL/ 
  *            HC_XACTERR/  
  *            HC_BBLERR/  
  *            HC_DATATGLERR    
  */
HC_STATUS  USBH_HC_GetState(USB_OTG_CORE_HANDLE *pdev, uint8_t chnum)
{
  return pdev->host.HC_Status[chnum];
}

uint32_t USBH_HC_Init(USB_OTG_CORE_HANDLE *pdev,  
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps)
{
  uint32_t status = 0;

  pdev->host.hc[ch_num].dev_addr = dev_address;
  pdev->host.hc[ch_num].max_packet = mps;
  pdev->host.channel[ch_num] = ch_num;
  pdev->host.hc[ch_num].ep_type = ep_type;
  pdev->host.hc[ch_num].ep_num = epnum & 0x7F;
  pdev->host.hc[ch_num].ep_is_in = ((epnum & 0x80) == 0x80);
  pdev->host.hc[ch_num].speed = speed;

  status =  HCD_HC_Init (pdev , ch_num);

  return status;
}

static rt_err_t drv_reset_port(rt_uint8_t port)
{
    RT_DEBUG_LOG(RT_DEBUG_USB, ("reset port\n"));
    HCD_ResetPort(&USB_OTG_Core);
    return RT_EOK;
}

static int drv_pipe_xfer(upipe_t pipe, rt_uint8_t token, void *buffer, int nbytes, int timeout)
{
    rt_err_t ret;
    HC_STATUS hc_stat;
    URB_STATE urb_stat;
    while (1)
    {
        if (!connect_status)
        {
            return -1;
        }
        rt_completion_init(&urb_completion);
        USBH_HC_SubmitRequest(&USB_OTG_Core,
                                 pipe->pipe_index,
                                 (pipe->ep.bEndpointAddress & 0x80) >> 7,
                                 pipe->ep.bmAttributes,
                                 token,
                                 buffer,
                                 nbytes,
                                 0);
        ret = rt_completion_wait(&urb_completion, timeout);
        if (ret < 0)
        {
            rt_kprintf("urb %s timeout\n", 
                (pipe->ep.bEndpointAddress & 0x80)? "in" : "out");
        }

        hc_stat = HCD_GetHCState(&USB_OTG_Core, pipe->pipe_index);
        urb_stat = HCD_GetURB_State(&USB_OTG_Core, pipe->pipe_index);
        if (hc_stat == HC_NAK)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("nak\n"));
            //rt_kprintf("nak\n");
            if (pipe->ep.bmAttributes == USB_EP_ATTR_INT)
            {
                rt_thread_delay((pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) > 0 ? (pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) : 1);
            }
            /*USB_OTG_HC_Halt(&USB_OTG_Core, pipe->pipe_index);
            USBH_HC_Init(&USB_OTG_Core,
                            pipe->pipe_index,
                            pipe->ep.bEndpointAddress,
                            pipe->inst->address,
                            USB_OTG_SPEED_FULL,
                            pipe->ep.bmAttributes,
                            pipe->ep.wMaxPacketSize);*/
            continue;
        }
        else if (hc_stat == HC_STALL)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("stall\n"));
            rt_kprintf("stall\n");
            pipe->status = UPIPE_STATUS_STALL;
            if (pipe->callback != RT_NULL)
            {
                pipe->callback(pipe);
            }
            return -1;
        }
        else if (hc_stat == HC_XACTERR || 
                 hc_stat == HC_BBLERR || 
                 hc_stat == HC_DATATGLERR)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("error\n"));
            rt_kprintf("error\n");
            pipe->status = UPIPE_STATUS_ERROR;
            if (pipe->callback != RT_NULL)
            {
                pipe->callback(pipe);
            }
            return -1;
        }
        else if (urb_stat == URB_DONE) //URB_NOTREADY URB_ERROR URB_NYET
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("ok\n"));
            pipe->status = UPIPE_STATUS_OK;
            if (pipe->callback != RT_NULL)
            {
                pipe->callback(pipe);
            }
            if (pipe->ep.bEndpointAddress & 0x80) {
                return HCD_GetXferCnt(&USB_OTG_Core, pipe->pipe_index);
            } 
            return nbytes;
        }
        else
        {
            rt_kprintf("URB xfer err, hc_stat=%d, urb_stat=%d\n", hc_stat, urb_stat);
        }
        return -1;
    }
}

static rt_uint16_t pipe_index = 0;
static rt_uint8_t  drv_get_free_pipe_index()
{
    rt_uint8_t idx;
    for (idx = 1; idx < 16; idx++)
    {
        if (!(pipe_index & (0x01 << idx)))
        {
            pipe_index |= (0x01 << idx);
            return idx;
        }
    }
    return 0xff;
}

static void drv_free_pipe_index(rt_uint8_t index)
{
    pipe_index &= ~(0x01 << index);
}

static rt_err_t drv_open_pipe(upipe_t pipe)
{
    pipe->pipe_index = drv_get_free_pipe_index();
    USBH_HC_Init(&USB_OTG_Core,
                    pipe->pipe_index,
                    pipe->ep.bEndpointAddress,
                    pipe->inst->address,
                    USB_OTG_SPEED_FULL,
                    pipe->ep.bmAttributes,
                    pipe->ep.wMaxPacketSize);
    /* Set DATA0 PID token*/
    if (USB_OTG_Core.host.hc[pipe->pipe_index].ep_is_in)
    {
        USB_OTG_Core.host.hc[pipe->pipe_index].toggle_in = 0;
    }
    else
    {
        USB_OTG_Core.host.hc[pipe->pipe_index].toggle_out = 0;
    }
    return RT_EOK;
}

static rt_err_t drv_close_pipe(upipe_t pipe)
{
    USB_OTG_HC_Halt(&USB_OTG_Core, pipe->pipe_index);
    drv_free_pipe_index(pipe->pipe_index);
    return RT_EOK;
}

struct uhcd_ops _uhcd_ops =
{
    drv_reset_port,
    drv_pipe_xfer,
    drv_open_pipe,
    drv_close_pipe,
};


/**
 * This function will initialize usbh host controller device.
 *
 * @param dev the host controller device to be initalize.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t usbh_init(rt_device_t dev)
{
    /* Hardware Init */
    USB_OTG_BSP_Init(&USB_OTG_Core);

    /* configure GPIO pin used for switching VBUS power */
    USB_OTG_BSP_ConfigVBUS(0);

    /* Host de-initializations */
    USBH_DeInit(&USB_OTG_Core, &USB_Host);

    /* Start the USB OTG core */
    HCD_Init(&USB_OTG_Core , USB_OTG_FS_CORE_ID);

    USBH_DeAllocate_AllChannel(&USB_OTG_Core);

    /* Enable Interrupts */
    USB_OTG_BSP_EnableInterrupt(&USB_OTG_Core);

    return RT_EOK;
}

/**
 * This function will define the usbh host controller device, it will be register to the device
 * system.
 *
 * @return the error code, RT_EOK on successfully.
 */
void rt_hw_usbh_init(void)
{
    stm32_hcd.parent.type = RT_Device_Class_USBHost;
    stm32_hcd.parent.init = usbh_init;
    stm32_hcd.parent.user_data = &USB_OTG_Core;
    stm32_hcd.ops = &_uhcd_ops;

    stm32_hcd.num_ports = 1;

    rt_device_register(&stm32_hcd.parent, "usbh", 0);
    rt_usb_host_init();
}

INIT_DEVICE_EXPORT(rt_hw_usbh_init);


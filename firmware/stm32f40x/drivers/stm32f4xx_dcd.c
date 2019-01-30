/**
 * USB device controller driver for RT-Thread RTOS
 *
 */
#include <rtthread.h>
#include <stm32f4xx.h>
#include <rtdevice.h>
#include "usb_core.h"
#include "usb_dcd.h"
#include "usb_dcd_int.h"
#include "usb_bsp.h"

static struct udcd stm32_dcd;
ALIGN(4) static USB_OTG_CORE_HANDLE USB_OTG_Core;

static struct ep_id _ep_pool[] =
{
    {0x0,  USB_EP_ATTR_CONTROL,     USB_DIR_INOUT,  64, ID_ASSIGNED  },
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0xFF, USB_EP_ATTR_TYPE_MASK,   USB_DIR_MASK,   0,  ID_ASSIGNED  },
};

void OTG_FS_IRQHandler(void)
{
    extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

    /* enter interrupt */
    rt_interrupt_enter();

    USBD_OTG_ISR_Handler (&USB_OTG_Core);

    rt_interrupt_leave();
}

/**
* @brief  USBD_SetupStage
*         Handle the setup stage
* @param  pdev: device instance
* @retval status
*/
static uint8_t USBD_SetupStage(USB_OTG_CORE_HANDLE *pdev)
{
    rt_usbd_ep0_setup_handler(&stm32_dcd, (struct urequest *)pdev->dev.setup_packet);

    return 0;
}

static uint8_t USBD_Reset(USB_OTG_CORE_HANDLE  *pdev)
{
    rt_kprintf("USBD_Reset\n");

    /* Open EP0 OUT */
    DCD_EP_Open(pdev,
                0x00,
                USB_OTG_MAX_EP0_SIZE,
                EP_TYPE_CTRL);

    /* Open EP0 IN */
    DCD_EP_Open(pdev,
                0x80,
                USB_OTG_MAX_EP0_SIZE,
                EP_TYPE_CTRL);

    /* Upon Reset call usr call back */
    pdev->dev.device_status = USB_OTG_DEFAULT;

    rt_usbd_reset_handler(&stm32_dcd);

    return 0;
}

static uint8_t USBD_DataOutStage(USB_OTG_CORE_HANDLE *pdev , uint8_t epnum)
{
    if (epnum != 0)
    {
        rt_usbd_ep_out_handler(&stm32_dcd, epnum,pdev->dev.out_ep[epnum].xfer_count);
    }
    else
    {
        rt_usbd_ep0_out_handler(&stm32_dcd, pdev->dev.out_ep[0].xfer_count);
    }

    return 0;
}

static uint8_t USBD_DataInStage(USB_OTG_CORE_HANDLE *pdev , uint8_t epnum)
{
    if (epnum == 0)
    {
        rt_usbd_ep0_in_handler(&stm32_dcd);
    }
    else
    {
        rt_usbd_ep_in_handler(&stm32_dcd, 0x80 | epnum, pdev->dev.in_ep[epnum].xfer_count);
    }

    return 0;
}

static uint8_t USBD_SOF(USB_OTG_CORE_HANDLE  *pdev)
{
    rt_usbd_sof_handler(&stm32_dcd);

    return 0;
}

static rt_uint8_t USBD_Suspend(USB_OTG_CORE_HANDLE  *pdev)
{
    return 0;
}

static uint8_t USBD_Resume(USB_OTG_CORE_HANDLE  *pdev)
{
    return 0;
}

static uint8_t USBD_IsoINIncomplete(USB_OTG_CORE_HANDLE  *pdev)
{
    return 0;
}

static uint8_t USBD_IsoOUTIncomplete(USB_OTG_CORE_HANDLE  *pdev)
{
    return 0;
}

static uint8_t USBD_DevConnected(USB_OTG_CORE_HANDLE  *pdev)
{
    rt_usbd_connect_handler(&stm32_dcd);
    return 0;
}

static uint8_t USBD_DevDisconnected(USB_OTG_CORE_HANDLE  *pdev)
{
    rt_usbd_disconnect_handler(&stm32_dcd);
    return 0;
}


USBD_DCD_INT_cb_TypeDef USBD_DCD_INT_cb =
{
    USBD_DataOutStage,
    USBD_DataInStage,
    USBD_SetupStage,
    USBD_SOF,
    USBD_Reset,
    USBD_Suspend,
    USBD_Resume,
    USBD_IsoINIncomplete,
    USBD_IsoOUTIncomplete,
#ifdef VBUS_SENSING_ENABLED
    USBD_DevConnected,
    USBD_DevDisconnected,
#endif
};

USBD_DCD_INT_cb_TypeDef  *USBD_DCD_INT_fops = &USBD_DCD_INT_cb;

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    DCD_EP_Stall(&USB_OTG_Core, address);
    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
    DCD_EP_ClrStall(&USB_OTG_Core, address);
    return RT_EOK;
}

static rt_err_t _set_address(rt_uint8_t address)
{
    USB_OTG_Core.dev.device_address = address;
    DCD_EP_SetAddress(&USB_OTG_Core, address);

    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep)
{
    uep_desc_t ep_desc = ep->ep_desc;

    RT_ASSERT(ep != RT_NULL);

    DCD_EP_Open(&USB_OTG_Core, ep_desc->bEndpointAddress,
                ep_desc->wMaxPacketSize, ep_desc->bmAttributes);

    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);

    DCD_EP_Close(&USB_OTG_Core, ep->ep_desc->bEndpointAddress);

    return RT_EOK;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
    return size;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    DCD_EP_PrepareRx(&USB_OTG_Core, address, buffer, size);
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    DCD_EP_Tx(&USB_OTG_Core, address, buffer, size);
    return size;
}

static rt_err_t _ep0_send_status(void)
{
    DCD_EP_Tx(&USB_OTG_Core, 0x00, NULL, 0);
    return RT_EOK;
}

static rt_err_t _suspend(void)
{
    return RT_EOK;
}

static rt_err_t _wakeup(void)
{
    return RT_EOK;
}

const static struct udcd_ops _udc_ops =
{
    _set_address,
    _set_config,
    _ep_set_stall,
    _ep_clear_stall,
    _ep_enable,
    _ep_disable,
    _ep_read_prepare,
    _ep_read,
    _ep_write,
    _ep0_send_status,
    _suspend,
    _wakeup,
};


static rt_err_t stm32_dcd_init(rt_device_t device)
{
    rt_kprintf("stm32_dcd_init\n");

    /* Hardware Init */
    USB_OTG_BSP_Init(&USB_OTG_Core);

    /* set USB OTG core params */
    DCD_Init(&USB_OTG_Core , USB_OTG_FS_CORE_ID);

    /* Enable Interrupts */
    USB_OTG_BSP_EnableInterrupt(&USB_OTG_Core);

    return RT_EOK;
}

int rt_hw_usbd_init(void)
{
    stm32_dcd.parent.type = RT_Device_Class_USBDevice;
    stm32_dcd.parent.init = stm32_dcd_init;
    stm32_dcd.parent.user_data = &USB_OTG_Core;

    stm32_dcd.ops = &_udc_ops;
    stm32_dcd.ep_pool = _ep_pool;
    stm32_dcd.ep0.id = &_ep_pool[0];

    rt_device_register(&stm32_dcd.parent, "usbd", 0);
    rt_usb_device_init();

    return 0;
}

INIT_DEVICE_EXPORT(rt_hw_usbd_init);


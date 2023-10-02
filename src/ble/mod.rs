//! Bad code ahead ... just copied some bits and pieces from the bare-metal / no-std implementation
//! Not really a good fit - should get a re-implementation most probably

use std::{cell::RefCell, mem::MaybeUninit};

use critical_section::Mutex;
use embedded_io::{
    blocking::{Read, Write},
    Error, Io,
};
use heapless::spsc::Queue as SimpleQueue;

static VHCI_HOST_CALLBACK: esp_idf_sys::esp_vhci_host_callback =
    esp_idf_sys::esp_vhci_host_callback {
        notify_host_send_available: Some(notify_host_send_available),
        notify_host_recv: Some(notify_host_recv),
    };

extern "C" fn notify_host_send_available() {}

extern "C" fn notify_host_recv(data: *mut u8, len: u16) -> i32 {
    unsafe {
        let mut buf = [0u8; 256];
        buf[..len as usize].copy_from_slice(&core::slice::from_raw_parts(data, len as usize));

        let packet = ReceivedPacket {
            len: len as u8,
            data: buf,
        };

        critical_section::with(|cs| {
            let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
            if queue.enqueue(packet).is_err() {}
        });

        asynch::hci_read_data_available();
    }

    0
}

pub fn ble_init() {
    let mut bt_cfg: esp_idf_sys::esp_bt_controller_config_t =
        esp_idf_sys::esp_bt_controller_config_t {
            controller_task_stack_size: esp_idf_sys::ESP_TASK_BT_CONTROLLER_STACK as u16,
            controller_task_prio: esp_idf_sys::ESP_TASK_BT_CONTROLLER_PRIO as u8,
            hci_uart_no: esp_idf_sys::BT_HCI_UART_NO_DEFAULT as u8,
            hci_uart_baudrate: esp_idf_sys::BT_HCI_UART_BAUDRATE_DEFAULT as u32,
            scan_duplicate_mode: esp_idf_sys::SCAN_DUPLICATE_MODE as u8,
            scan_duplicate_type: esp_idf_sys::SCAN_DUPLICATE_TYPE_VALUE as u8,
            normal_adv_size: esp_idf_sys::NORMAL_SCAN_DUPLICATE_CACHE_SIZE as u16,
            mesh_adv_size: esp_idf_sys::MESH_DUPLICATE_SCAN_CACHE_SIZE as u16,
            send_adv_reserved_size: esp_idf_sys::SCAN_SEND_ADV_RESERVED_SIZE as u16,
            controller_debug_flag: esp_idf_sys::CONTROLLER_ADV_LOST_DEBUG_BIT as u32,
            mode: 1,
            ble_max_conn: esp_idf_sys::CONFIG_BTDM_CTRL_BLE_MAX_CONN_EFF as u8,
            bt_max_acl_conn: esp_idf_sys::CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN_EFF as u8,
            bt_sco_datapath: esp_idf_sys::CONFIG_BTDM_CTRL_BR_EDR_SCO_DATA_PATH_EFF as u8,
            auto_latency: esp_idf_sys::BTDM_CTRL_AUTO_LATENCY_EFF != 0,
            bt_legacy_auth_vs_evt: esp_idf_sys::BTDM_CTRL_LEGACY_AUTH_VENDOR_EVT_EFF != 0,
            bt_max_sync_conn: esp_idf_sys::CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF as u8,
            ble_sca: esp_idf_sys::CONFIG_BTDM_BLE_SLEEP_CLOCK_ACCURACY_INDEX_EFF as u8,
            pcm_role: esp_idf_sys::CONFIG_BTDM_CTRL_PCM_ROLE_EFF as u8,
            pcm_polar: esp_idf_sys::CONFIG_BTDM_CTRL_PCM_POLAR_EFF as u8,
            hli: esp_idf_sys::BTDM_CTRL_HLI != 0,
            dup_list_refresh_period: esp_idf_sys::SCAN_DUPL_CACHE_REFRESH_PERIOD as u16,
            magic: esp_idf_sys::ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL as u32,
        };

    unsafe {
        esp_idf_sys::esp!(esp_idf_sys::nvs_flash_init()).unwrap();
        esp_idf_sys::esp!(esp_idf_sys::esp_bt_controller_init(&mut bt_cfg)).unwrap();
        esp_idf_sys::esp!(esp_idf_sys::esp_bt_controller_enable(1)).unwrap();
    }
}

#[non_exhaustive]
pub struct BleConnector {}

impl<'d> BleConnector {
    pub fn new() -> BleConnector {
        unsafe {
            esp_idf_sys::esp_vhci_host_register_callback(
                &VHCI_HOST_CALLBACK as *const esp_idf_sys::esp_vhci_host_callback,
            );
        }
        Self {}
    }
}

#[derive(Debug)]
pub enum BleConnectorError {}

impl Error for BleConnectorError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl Io for BleConnector {
    type Error = BleConnectorError;
}

impl Read for BleConnector {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut total = 0;
        for b in buf {
            let mut buffer = [0u8];
            let len = read_hci(&mut buffer);

            if len == 1 {
                *b = buffer[0];
                total += 1;
            } else {
                return Ok(total);
            }
        }

        Ok(total)
    }
}

impl Write for BleConnector {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for b in buf {
            send_hci(&[*b]);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // nothing to do
        Ok(())
    }
}

static BT_RECEIVE_QUEUE: Mutex<RefCell<SimpleQueue<ReceivedPacket, 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

static mut BLE_HCI_READ_DATA: [u8; 256] = [0u8; 256];
static mut BLE_HCI_READ_DATA_INDEX: usize = 0;
static mut BLE_HCI_READ_DATA_LEN: usize = 0;

static mut HCI_OUT_COLLECTOR: MaybeUninit<HciOutCollector> = MaybeUninit::uninit();

#[derive(PartialEq, Debug)]
enum HciOutType {
    Unknown,
    Acl,
    Command,
}

struct HciOutCollector {
    data: [u8; 256],
    index: usize,
    ready: bool,
    kind: HciOutType,
}

impl HciOutCollector {
    fn _new() -> HciOutCollector {
        HciOutCollector {
            data: [0u8; 256],
            index: 0,
            ready: false,
            kind: HciOutType::Unknown,
        }
    }

    fn is_ready(&self) -> bool {
        self.ready
    }

    fn push(&mut self, data: &[u8]) {
        self.data[self.index..(self.index + data.len())].copy_from_slice(data);
        self.index += data.len();

        if self.kind == HciOutType::Unknown {
            self.kind = match self.data[0] {
                1 => HciOutType::Command,
                2 => HciOutType::Acl,
                _ => HciOutType::Unknown,
            };
        }

        if !self.ready {
            if self.kind == HciOutType::Command && self.index >= 4 {
                if self.index == self.data[3] as usize + 4 {
                    self.ready = true;
                }
            } else if self.kind == HciOutType::Acl && self.index >= 5 {
                if self.index == (self.data[3] as usize) + ((self.data[4] as usize) << 8) + 5 {
                    self.ready = true;
                }
            }
        }
    }

    fn reset(&mut self) {
        self.index = 0;
        self.ready = false;
        self.kind = HciOutType::Unknown;
    }

    fn packet(&self) -> &[u8] {
        &self.data[0..(self.index as usize)]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ReceivedPacket {
    pub len: u8,
    pub data: [u8; 256],
}

pub fn read_hci(data: &mut [u8]) -> usize {
    unsafe {
        if BLE_HCI_READ_DATA_LEN == 0 {
            critical_section::with(|cs| {
                let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

                if let Some(packet) = queue.dequeue() {
                    BLE_HCI_READ_DATA[..packet.len as usize]
                        .copy_from_slice(&packet.data[..packet.len as usize]);
                    BLE_HCI_READ_DATA_LEN = packet.len as usize;
                    BLE_HCI_READ_DATA_INDEX = 0;
                }
            });
        }

        if BLE_HCI_READ_DATA_LEN > 0 {
            data[0] = BLE_HCI_READ_DATA[BLE_HCI_READ_DATA_INDEX];
            BLE_HCI_READ_DATA_INDEX += 1;

            if BLE_HCI_READ_DATA_INDEX >= BLE_HCI_READ_DATA_LEN {
                BLE_HCI_READ_DATA_LEN = 0;
                BLE_HCI_READ_DATA_INDEX = 0;
            }
            return 1;
        }
    }

    0
}

pub fn send_hci(data: &[u8]) {
    let hci_out = unsafe { &mut *HCI_OUT_COLLECTOR.as_mut_ptr() };
    hci_out.push(data);

    if hci_out.is_ready() {
        let packet = hci_out.packet();

        unsafe {
            loop {
                let can_send = esp_idf_sys::esp_vhci_host_check_send_available();

                if !can_send {
                    continue;
                }

                esp_idf_sys::esp_vhci_host_send_packet(
                    packet.as_ptr() as *mut u8,
                    packet.len() as u16,
                );

                break;
            }
        }

        hci_out.reset();
    }
}

pub fn have_hci_read_data() -> bool {
    critical_section::with(|cs| {
        let queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
        !queue.is_empty()
            || unsafe {
                BLE_HCI_READ_DATA_LEN > 0 && (BLE_HCI_READ_DATA_LEN >= BLE_HCI_READ_DATA_INDEX)
            }
    })
}

pub mod asynch {
    use core::task::Poll;

    use super::have_hci_read_data;
    use super::VHCI_HOST_CALLBACK;

    use super::BleConnectorError;
    use super::{read_hci, send_hci};
    use embassy_sync::waitqueue::AtomicWaker;
    use embedded_io::asynch;
    use embedded_io::Io;

    static HCI_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) fn hci_read_data_available() {
        HCI_WAKER.wake();
    }

    #[non_exhaustive]
    pub struct BleConnector {}

    impl<'d> BleConnector {
        pub fn new() -> BleConnector {
            unsafe {
                esp_idf_sys::esp_vhci_host_register_callback(
                    &VHCI_HOST_CALLBACK as *const esp_idf_sys::esp_vhci_host_callback,
                );
            }

            Self {}
        }
    }

    impl Io for BleConnector {
        type Error = BleConnectorError;
    }

    impl asynch::Read for BleConnector {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
            if !have_hci_read_data() {
                HciReadyEventFuture.await;
            }

            let mut total = 0;
            for b in buf {
                let mut buffer = [0u8];
                let len = read_hci(&mut buffer);

                if len == 1 {
                    *b = buffer[0];
                    total += 1;
                } else {
                    return Ok(total);
                }
            }

            Ok(total)
        }
    }

    impl asynch::Write for BleConnector {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
            send_hci(buf);
            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), BleConnectorError> {
            // nothing to do
            Ok(())
        }
    }

    pub(crate) struct HciReadyEventFuture;

    impl core::future::Future for HciReadyEventFuture {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            HCI_WAKER.register(cx.waker());

            if have_hci_read_data() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}

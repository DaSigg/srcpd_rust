use std::sync::mpsc::{Receiver, Sender};

use gpio::sysfs::SysFsGpioInput;

use crate::srcp_protocol_ddl::SmReadWrite;

/// Input Prog Ack Signal GPIO 22 (= Pin 15, RI von RS232)
const GPIO_PROG_ACK: u16 = 22;

/// Read / Write für DccCvTel
#[derive(PartialEq)]
pub enum MfxCvTelType {
  /// Ver. ein Bit. Bitnummer und Bit Value
  VerifyBit(u8, bool),
  /// Ver. ein Byte.
  VerifyByte(u8),
  /// Write ein Byte
  WriteByte(u8),
}

/// DCC CV Read/Write Telegramm senden durch DDL DCC Anfordern
pub struct DccCvTel {
  /// Lok Adrdesse, 0 bei Prog.Gleis
  pub adr: u32,
  /// Lesen oder Schreiben?
  pub mfx_cv_type: MfxCvTelType,
  /// CV (10 Bits)
  pub cv: u16,
}

/// Thread zur Ausführung DCC Dekoder Prog. Read/Write/Verify Befehlen inkl. Rückmeldungen Prog.Gleis.
/// Abarbeitung der Aufträge.
/// - Aufträge werden empfangen aus DDL Thread, DCC Protokoll
/// - Notwendige Telegramme werden zurück an DDL Thread, DCC Protokoll gesandt, da die
///   SPI Ausgabe von da aus geschehen muss.
/// - Antworten werden zurück gesendet.
///   Es erfolgt immer eine Antwort auf eine Anfrage, im Fehlerfalle "Error".
pub struct DccProgThread {
  /// GPIO zum Einlesen Quittungsimpuls
  gpio_prog_ack: SysFsGpioInput,
  /// Receiver für Aufträge
  rx: Receiver<SmReadWrite>,
  /// Sender für Ergenisse der Aufträge, als Antwort auf "ReadCV"/"WriteCV"/"Verify"
  tx: Sender<SmReadWrite>,
  /// Sender für über SPI zu versendende Telegramme
  tx_tel: Sender<DccCvTel>,
}

impl DccProgThread {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * rx - Empfang von Aufträge.
  /// * tx - Sender zum versenden er eingelesen Rückmeldungen als Antwort auf "ReadCA"/"WriteCA"
  /// * tx_tel - Sender zum versenden von auszugebenden Telegrammen
  pub fn new(
    rx: Receiver<SmReadWrite>, tx: Sender<SmReadWrite>, tx_tel: Sender<DccCvTel>,
  ) -> DccProgThread {
    DccProgThread {
      gpio_prog_ack: SysFsGpioInput::open(GPIO_PROG_ACK)
        .expect("GPIO_MFX_RDS_QAL konnte nicht geöffnet werden"),
      rx,
      tx,
      tx_tel,
    }
  }

  /// Als Thread ausführen
  pub fn execute(&mut self) {
    loop {}
  }
}

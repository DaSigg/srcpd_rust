use std::{
  sync::mpsc::{Receiver, Sender},
  thread,
  time::{Duration, Instant},
};

use gpio_cdev::{Chip, LineHandle, LineRequestFlags};
use log::{error, debug, warn, info};

use crate::srcp_protocol_ddl::{SmReadWrite, SmReadWriteType};

/// SRCP Type für CV Byte Zugriff
pub static DCC_SM_TYPE_CV: &str = "CV";
pub static DCC_SM_TYPE_CVBIT: &str = "CVBIT";

/// Input Prog Ack Signal GPIO 22 (= Pin 15, RI von RS232)
const GPIO_PROG_ACK: u32 = 22;

/// Timeout für Quittierungsimpuls vom Dekoder, 100ms mit Reserve weil Timeout mit versenden startet,
/// 5 * Prog Befehl senden dauert auch ca. 60 ms.
const DEC_ACK_TIMEOUT: Duration = Duration::from_millis(200);

/// Read / Write für DccCvTel
#[derive(PartialEq, Clone, Debug)]
pub enum DccCvTelType {
  /// Ver. ein Bit. (Bit Value, Bitnummer), nur Prog Gleis.
  VerifyBit(bool, u8),
  /// Ver. ein Byte (value), nur Prog Gleis.
  VerifyByte(u8),
  /// Write ein Byte, (value, prog_gleis), wenn prog_gleis=false, dann ist es Hauptgleisprogrammierung
  WriteByte(u8, bool),
  /// Write ein Bit, (value, Bitnr, prog_gleis), wenn prog_gleis=false, dann ist es Hauptgleisprogrammierung
  WriteBit(bool, u8, bool),
}

/// DCC CV Read/Write Telegramm senden durch DDL DCC Anfordern
#[derive(Clone, Debug)]
pub struct DccCvTel {
  /// Lok Adrdesse, 0 bei Prog.Gleis
  pub adr: u32,
  /// Lesen oder Schreiben?
  pub dcc_cv_type: DccCvTelType,
  /// CV (10 Bits)
  pub cv: u16,
  /// Oszi Trigger?
  pub trigger: bool,
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
  gpio_prog_ack: LineHandle,
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
      gpio_prog_ack: Chip::new("/dev/gpiochip0").expect("/dev/gpiochip0 konnte nicht geöffnet werden").
        get_line(GPIO_PROG_ACK).expect("GPIO_MFX_RDS_QAL konnte nicht geöffnet werden").
        request(LineRequestFlags::INPUT, 0, "input_dcc_prog_ack").expect("GPIO_MFX_RDS_QAL konnte nicht als Input geöffnet werden"),
      rx,
      tx,
      tx_tel,
    }
  }

  /// Senden eines CV Write/Verify Kommandos.
  /// Wenn "prog_gleis" wird bei Dekoder Quittierung true, sonst false zurück geliefert.
  /// Bei einem Fehler, wenn Quittung bereits vor Befehl ansteht, wird None zurück geliefert
  /// Wenn kein "prog_gleis" wird immer true zurück geliefert
  /// # Arguments
  /// * dcc_cv_tel - zu sendendes CV Telegramm.
  /// * prog_gleis - true wenn Prog Gleis und Dekoder Quittierung erwartet wird.
  fn send_dcc_cv_tel(&mut self, dcc_cv_tel: &DccCvTel, prog_gleis: bool) -> Option<bool> {
    debug!("DccProgThread tx_tel dcc_cv_tel={:?} prog_gleis={}", dcc_cv_tel, prog_gleis);
    let ack_vorher = self.gpio_prog_ack.get_value().unwrap() == 1;
    self.tx_tel.send(dcc_cv_tel.clone()).unwrap();
    if prog_gleis {
      let mut ack = Some(false);
      //Warten auf Quittierungsimpuls. Dieser sollte nach spätestens 100ms vorhanden sein und min. 5ms lang sein.
      let timeout = Instant::now();
      while (timeout + DEC_ACK_TIMEOUT) > Instant::now() {
        //Impuls ist sicher 5ms lang, also reicht es, alle 0.5ms zu prüfen
        thread::sleep(Duration::from_micros(500));
        if self.gpio_prog_ack.get_value().unwrap() == 1 {
          //Immer ganzen Timeout warten auch wenn Impuls erkannt wurde.
          //Grund: Prog. Paket muss 5 mal gesendet werden, Dekoder darf aber nach 2. Paket antworten.
          //Damit kann er in einem 5er Paket zweimal Antworten und es muss vermieden werden, dass
          //zweite Antwort als Antwort auf eventuell nächsten Befehl interpretiert wird.
          //Quittierung, wenn vorher Quittierung auch schon anstand ist das falsch
          if ack_vorher {
            warn!("DccProgThread send_dcc_cv_tel Dekoder Quittierung vorher anstehend");
            ack = None;
          }
          else {
            info!("DccProgThread send_dcc_cv_tel Dekoder Quittierung OK");
            ack = Some(true);
          }
        }
      }
      debug!("DccProgThread send_dcc_cv_tel Dekoder Quittierung: {:?}", ack);
      return ack;
    } else {
      return Some(true);
    }
  }

  /// Führt ein SM Kommando für Write und Verify aus.
  /// Wenn Programmiergleis: liefert true zrück wenn Dekoder Quittierung empfangen wurde, sonst false.
  /// Bei einem Fehler wird None geliefert.
  /// Wenn Hauptgleis: nur Write wird unterstützt, dann ist Ergebnis immer true, bei allen anderen Kommandos immer false.
  /// # Arguments
  /// * smcmd - Auszuführendes SM Kommando.
  fn execute_sm_cmd_write_ver(&mut self, smcmd: &SmReadWrite) -> Option<bool> {
    let mut result = None;
    //Bei Prog Gleis geht Write und Verify, sonst nur Write
    if smcmd.prog_gleis || matches!(smcmd.val, SmReadWriteType::Write(_)) {
      let dcc_cv_type = match smcmd.val {
        SmReadWriteType::Write(val) => Some(if smcmd.sm_type == DCC_SM_TYPE_CV {
          //CV
          DccCvTelType::WriteByte(val as u8, smcmd.prog_gleis)
        } else {
          //CVBIT, 2. Parameter ist Bitnr
          DccCvTelType::WriteBit(val != 0, smcmd.para[1] as u8, smcmd.prog_gleis)
        }),
        SmReadWriteType::Verify(val) => Some(if smcmd.sm_type == DCC_SM_TYPE_CV {
          //CV
          DccCvTelType::VerifyByte(val as u8)
        } else {
          //CVBIT, 2. Parameter ist Bitnr
          DccCvTelType::VerifyBit(val != 0, smcmd.para[1] as u8)
        }),
        _ => None, //Alles andere ist falsch
      };
      if let Some(dcc_cv_type) = dcc_cv_type {
        result = self.send_dcc_cv_tel(
          &DccCvTel {
            adr: smcmd.adr,
            dcc_cv_type,
            cv: smcmd.para[0] as u16, //Erster Parameter muss CV sein
            trigger: smcmd.trigger,
          },
          smcmd.prog_gleis,
        );
      }
    }
    result
  }

  /// Ein CV Bit mittels Verify auslesen.
  /// Liefert 1 wenn Bit gesetzt, 0 wenn nicht gesetzt zurück, None bei Fehler.
  /// Das Bit wird mit 0 und 1 verifiziert, bei einem wird Quittung erwartet, beim anderen dann nicht.
  /// # Arguments
  /// * adr - GL Dekoderadresse.
  /// * cv - CV Nr 1 bis 1024
  /// * bitnr - Die Bitnr 0 bis 7 des auszulesenden Bits
  /// * trigger - Oszi Trigger?
  fn read_cv_bit(&mut self, adr: u32, cv: u16, bitnr: u8, trigger: bool) -> Option<u8> {
    //Zuerst Ver Bit mit 0
    let mut dcc_cv_tel: DccCvTel = DccCvTel { adr, dcc_cv_type: DccCvTelType::VerifyBit(false, bitnr), cv, trigger };
    let result_bit0 = self.send_dcc_cv_tel(&dcc_cv_tel, true);
    //Dann Ver Bit mit 1
    dcc_cv_tel.dcc_cv_type = DccCvTelType::VerifyBit(true, bitnr);
    let result_bit1 = self.send_dcc_cv_tel(&dcc_cv_tel, true);
    if result_bit0.is_none() || result_bit1.is_none() {
      warn!("DccProgThread read_cv_bit Error. adr={}, CV={}, Bitnr={}, bit0={:?}, bit1={:?}", adr, cv, bitnr, result_bit0, result_bit1);
      None
    }
    else {
      //Wenn nun beide false oder beide true, dann konnte das Bit nicht korrekt gelesen werden
      if result_bit0.unwrap() ^ result_bit1.unwrap() {
        info!("DccProgThread read_cv_bit OK. adr={}, CV={}, Bitnr={}, bit={}", adr, cv, bitnr, result_bit0.unwrap());
        Some(result_bit1.unwrap() as u8)
      }
      else {
        warn!("DccProgThread read_cv_bit Dekoder Quittung bei 0 und 1 gleich. adr={}, CV={}, Bitnr={}, bit0={:?}, bit1={:?}", adr, cv, bitnr, result_bit0, result_bit1);
        None
      }
    }
  }

  /// Ein CV (Byte oder Bit) mittels Verify von einzelnen Bits auslesen.
  /// Liefert den ausgelesen Wert zurück, None bei Fehler
  /// Jedes Bit wird mit 0 und 1 verifiziert, bei einem wird Quittung erwartet, beim anderen dann nicht.
  /// Wenn ganzes CV Byte ausgelesen wird, wird schlussendlich wird noch das ganze Byte verifiziert.
  /// # Arguments
  /// * smcmd - Auszuführendes GET SM Kommando.
  fn read_cv(&mut self, smcmd: &SmReadWrite) -> Option<u8> {
    let cv = smcmd.para[0] as u16;
    if smcmd.sm_type == DCC_SM_TYPE_CV {
      //Ganzes CV Byte, alle Bits durchgehen
      let mut result: u8 = 0;
      for bitnr in 0..=7 {
        if let Some(bitval) = self.read_cv_bit(smcmd.adr, cv, bitnr, smcmd.trigger) {
          result |= bitval << bitnr;
        }
        else {
          //Abbruch, Fehler, Bit konnt nicht gelesen werden
          warn!("DccProgThread read_cv Byte Error. smcmd={:?}, bitnr={}", smcmd, bitnr);
          return None;
        }
      }
      //Nun noch ganzes Byte verifizieren
      let mut sm_ver_cmd = smcmd.clone();
      sm_ver_cmd.val = SmReadWriteType::Verify(result as u32);
      if let Some(ver_result) = self.execute_sm_cmd_write_ver(&sm_ver_cmd) {
        if ver_result {
          debug!("DccProgThread read_cv Byte OK. smcmd={:?}, CV={}", smcmd, result);
          return Some(result);
        }
        else {
          debug!("DccProgThread read_cv Byte Error. smcmd={:?}, CV={}", smcmd, result);
          return None;
        }
      }
      else {
        warn!("DccProgThread read_cv Byte Error. smcmd={:?}, CV={}", smcmd, result);
        return None;
      }
    } else {
      //CVBIT
      let bitnr = smcmd.para[1] as u8;
      return self.read_cv_bit(smcmd.adr, cv, bitnr, smcmd.trigger);
    }
  }

  /// Als Thread ausführen
  /// Thread wäre eigentlich für Write und Verify Kommandos nicht notwendig.
  /// Aber für GET schon, da dies mit Verify von einzelnen Bits gemacht werden muss.
  /// Deshalb als Thread und bei allen Befehlen gleich.
  pub fn execute(&mut self) {
    loop {
      let mut smcmd = self.rx.recv().unwrap();
      debug!("DccProgThread neues SM Kommando: {:?}", smcmd);
      //Default = Fehler
      let mut ans = SmReadWriteType::ResultErr;
      //Gültigkeit der Parameter prüfen
      //Write und Ver haben Value, 0 ist immer gültig als Default
      let val = match smcmd.val{
        SmReadWriteType::Write(val) | SmReadWriteType::Verify(val) => val,
        _ => 0
      };
      let para_valid = 
        //Type: CV oder CVBIT
        ((smcmd.sm_type == DCC_SM_TYPE_CV) || (smcmd.sm_type == DCC_SM_TYPE_CVBIT)) &&
        //CV: 1 bis 1024
        ((smcmd.para[0] >= 1) || (smcmd.para[0] <= 1024)) &&
        if smcmd.sm_type == DCC_SM_TYPE_CVBIT {
          //Value: bei CVBIT 0 oder 1
          (val <= 1) &&
          //Bei CVBIT: Bitnr 0 bis 7
          (smcmd.para[1] <= 7)
        } else {
          //Value: bei CV 0 bis 255
          val <= 255
        };
      if para_valid {
        match smcmd.val {
          SmReadWriteType::Read => {
            if let Some(val) = self.read_cv(&smcmd) {
              //Erfolgreich ausgelesen
              ans = SmReadWriteType::ResultOk(val as u32);
            }
          }
          SmReadWriteType::Write(val) | SmReadWriteType::Verify(val) => {
            if let Some(result) = self.execute_sm_cmd_write_ver(&smcmd) {
              if result {
                //Erfolgreich ausgeführt
                ans = SmReadWriteType::ResultOk(val);
              }
            }
          }
          _ => {
            error!("DccProgThread ungültiges Kommando erhalten: {:?}", smcmd);
          }
        }
      }
      //Antwort zurücksenden
      smcmd.val = ans;
      debug!("DccProgThread Sende Antwort: {:?}", smcmd);
      self.tx.send(smcmd).unwrap();
    }
  }
}

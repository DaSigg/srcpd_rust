use gpio::{sysfs::SysFsGpioInput, GpioIn, GpioValue};
use log::{debug, info, warn};
use std::{
  cmp::min,
  collections::HashMap,
  sync::mpsc::{Receiver, Sender},
  thread,
  time::{Duration, Instant},
};

use crate::srcp_protocol_ddl::{SmReadWrite, SmReadWriteType};

/// Input RDS Qual Signal GPIO 23 (= Pin 16)
const GPIO_MFX_RDS_QAL: u16 = 23;
/// Input RDS Clk Signal GPIO 24 (= Pin 18)
const GPIO_MFX_RDS_CLK: u16 = 24;
/// Input RDS Daten Signal GPIO 25 (= Pin 22)
const GPIO_MFX_RDS_DAT: u16 = 25;

/// Anzahl MFX Funktionen
const MFX_FX_COUNT: usize = 16;

/// Alle möglichen MFX Dekoder Blocktypen
#[derive(Debug, PartialEq, Clone)]
#[allow(dead_code)]
enum BlockTypenE {
  BlockGrundeinstellungen = 1,
  BlockFunktionalitaet = 2,
  BlockAutofunktionen = 3,
  BlockFunktionMapping = 4,
  BlockFahr = 5,
  BlockAusgaenge = 6,
  BlockProtokolle = 7,
  BlockSound = 8,
  BlockOptionen = 9,
}
impl BlockTypenE {
  /// Liefert Blocktype als enum von einem Byte
  /// Liefert None, wenn ungültiger Wert
  /// # Arguments
  /// * val - Wert aus dem der enum erzeugt werden soll.
  fn from(val: u8) -> Option<BlockTypenE> {
    match val {
      x if x == BlockTypenE::BlockGrundeinstellungen as u8 => {
        Some(BlockTypenE::BlockGrundeinstellungen)
      }
      x if x == BlockTypenE::BlockFunktionalitaet as u8 => Some(BlockTypenE::BlockFunktionalitaet),
      x if x == BlockTypenE::BlockAutofunktionen as u8 => Some(BlockTypenE::BlockAutofunktionen),
      x if x == BlockTypenE::BlockFunktionMapping as u8 => Some(BlockTypenE::BlockFunktionMapping),
      x if x == BlockTypenE::BlockFahr as u8 => Some(BlockTypenE::BlockFahr),
      x if x == BlockTypenE::BlockAusgaenge as u8 => Some(BlockTypenE::BlockAusgaenge),
      x if x == BlockTypenE::BlockProtokolle as u8 => Some(BlockTypenE::BlockProtokolle),
      x if x == BlockTypenE::BlockSound as u8 => Some(BlockTypenE::BlockSound),
      x if x == BlockTypenE::BlockOptionen as u8 => Some(BlockTypenE::BlockOptionen),
      _ => None,
    }
  }
}

/// CA's für alle Blöcke
#[non_exhaustive]
#[derive(Debug, PartialEq)]
#[allow(dead_code)]
enum BlockCaE {
  //Blockbeschreibungen
  CaNichtVerwendet,
  CaBlockBeschreibung,
  //BLOCK_GRUNDEINSTELLUNGEN
  CaGrundHersteller,
  CaGrundKennung,
  CaGrundVersionb,
  CaGrundVersiona,
  CaGrundProtokollInfo,
  CaGrundLokid,
  CaGrundBlocktab,
  CaGrundLokname,
  CaGrundBenutzer,
  CaGrundVersionhw,
  //BLOCK_FUNKTIONALITAET
  CaFunkFahrfunktion,
  CaFunkSchaltfunktion,
  //BLOCK_AUTOFUNKTIONEN
  CaAutoSchaltfunktionStand,
  CaAutoSchaltfunktionFahr,
  //BLOCK_FUNKTION_MAPPING
  CaFmapFunktionSymbol,
  CaFmapFunktionVorwaerts,
  CaFmapFunktionRueckwaerts,
  //BLOCK_FAHR
  CaFahrMotoren,
  CaFahrMotortyp,
  CaFahrMotorfreq,
  CaFahrBeschBrems,
  CaFahrTrimm,
  CaFahrRegelung,
  CaFahrBremstrecke,
  CaFahrVtab,
  CaFahrTacho,
  CaFahrReverse,
  //BLOCK_AUSGAENGE
  CaAusgaengeKonfig,
  CaAusgaengeKonfigInt,
  CaAusgaengeKonfigSound,
  //BLOCK_PROTOKOLLE
  CaProtokolleProtokoll,
  CaProtokolleKonfig,
  CaProtokolleFunktionOn,
  CaProtokolleAdresseMmDcc,
  CaProtokolleAnalog,
  //BLOCK_SOUND
  CaSoundVolume,
  CaSoundTypDieselE,
  CaSoundTypDampf,
  CaSoundGeschwindigkeit,
  CaSoundZufall,
  CaSoundBrems,
  CaSoundAuto,
  //BLOCK_OPTIONEN
  CaOptDiv,
  CaOptRichtung,
}
impl BlockCaE {
  /// Liefert den eigentlichen Inhalt zurück. Tuple:
  ///  .0: Anzahl Bytes in diesem CA (ohne. CA ID)
  ///  .1: CA ID
  fn value(&self) -> (u8, u8) {
    match *self {
      BlockCaE::CaNichtVerwendet => (0x00, 0x00),
      BlockCaE::CaBlockBeschreibung => (0x05, 0x01),
      BlockCaE::CaGrundHersteller => (0x08, 0x10),
      BlockCaE::CaGrundKennung => (0x08, 0x11),
      BlockCaE::CaGrundVersionb => (0x08, 0x12),
      BlockCaE::CaGrundVersiona => (0x0C, 0x13),
      BlockCaE::CaGrundProtokollInfo => (0x10, 0x14),
      BlockCaE::CaGrundLokid => (0x08, 0x16),
      BlockCaE::CaGrundBlocktab => (0x3E, 0x17),
      BlockCaE::CaGrundLokname => (0x10, 0x18),
      BlockCaE::CaGrundBenutzer => (0x10, 0x19),
      BlockCaE::CaGrundVersionhw => (0x0C, 0x17),
      BlockCaE::CaFunkFahrfunktion => (0x01, 0x10),
      BlockCaE::CaFunkSchaltfunktion => (0x10, 0x11),
      BlockCaE::CaAutoSchaltfunktionStand => (0x01, 0x10),
      BlockCaE::CaAutoSchaltfunktionFahr => (0x01, 0x11),
      BlockCaE::CaFmapFunktionSymbol => (0x03, 0x10),
      BlockCaE::CaFmapFunktionVorwaerts => (0x04, 0x12),
      BlockCaE::CaFmapFunktionRueckwaerts => (0x04, 0x13),
      BlockCaE::CaFahrMotoren => (0x01, 0x10),
      BlockCaE::CaFahrMotortyp => (0x01, 0x11),
      BlockCaE::CaFahrMotorfreq => (0x02, 0x12),
      BlockCaE::CaFahrBeschBrems => (0x02, 0x13),
      BlockCaE::CaFahrTrimm => (0x02, 0x14),
      BlockCaE::CaFahrRegelung => (0x04, 0x15),
      BlockCaE::CaFahrBremstrecke => (0x01, 0x16),
      BlockCaE::CaFahrVtab => (0x1C, 0x17),
      BlockCaE::CaFahrTacho => (0x02, 0x18),
      BlockCaE::CaFahrReverse => (0x01, 0x19),
      BlockCaE::CaAusgaengeKonfig => (0x03, 0x10),
      BlockCaE::CaAusgaengeKonfigInt => (0x02, 0x11),
      BlockCaE::CaAusgaengeKonfigSound => (0x02, 0x12),
      BlockCaE::CaProtokolleProtokoll => (0x01, 0x10),
      BlockCaE::CaProtokolleKonfig => (0x01, 0x11),
      BlockCaE::CaProtokolleFunktionOn => (0x02, 0x12),
      BlockCaE::CaProtokolleAdresseMmDcc => (0x04, 0x13),
      BlockCaE::CaProtokolleAnalog => (0x02, 0x14),
      BlockCaE::CaSoundVolume => (0x01, 0x10),
      BlockCaE::CaSoundTypDieselE => (0x02, 0x11),
      BlockCaE::CaSoundTypDampf => (0x02, 0x12),
      BlockCaE::CaSoundGeschwindigkeit => (0x02, 0x13),
      BlockCaE::CaSoundZufall => (0x02, 0x14),
      BlockCaE::CaSoundBrems => (0x01, 0x15),
      BlockCaE::CaSoundAuto => (0x02, 0x16),
      BlockCaE::CaOptDiv => (0x01, 0x10),
      BlockCaE::CaOptRichtung => (0x01, 0x11),
    }
  }
}

/// Zustände RDS Empfang
#[derive(PartialEq)]
enum StateRdsRx {
  StateStart1,   //Initiale 1 Folge
  StateStart010, //In Startkennung nach 1 Folge
  StateData,     //Nutzdatenempfang
  StateCheck,    //Prüfung, CRC
  StateFinal,
}

/// Auftragtypen für "MfxRdsJob"
pub enum MfxRdsJob {
  //Antwort sind alle notwendigen Init Parameter (Lokname und Funktionen) über Sender "tx_lok_init"
  ReadAllInitParameter { adr: u32 },
  //Antwort ist gelesener/geschriebener Wert über "Sender "tx"
  ReadWriteCA { ca_parameter: SmReadWrite },
}
impl MfxRdsJob {
  /// Liefert MfxRdsJob ReadAllInitParameter
  /// # Arguments
  /// * adr - Lokadresse.
  pub fn new_read_all_init_parameter(adr: u32) -> MfxRdsJob {
    MfxRdsJob::ReadAllInitParameter { adr }
  }
}

/// Anzahl Bytes für MfxCvTel Read/Write
/// Bei Write kann nur 1 Byte verwendet werden, alles andere scheint nicht zu funktionieren, siehe auch
/// "Beschreibung des mfx®Schienenformats, Stefan Krauß"
#[derive(Clone)]
pub enum MfxCvTelBytes {
  Cc1byte,
  Cc2Byte,
  Cc4Byte,
  Cc8Byte,
}
impl MfxCvTelBytes {
  /// Liefert die Anzahl Bytes zurück
  pub fn byte_count(&self) -> usize {
    match self {
      MfxCvTelBytes::Cc1byte => 1,
      MfxCvTelBytes::Cc2Byte => 2,
      MfxCvTelBytes::Cc4Byte => 4,
      MfxCvTelBytes::Cc8Byte => 8,
    }
  }
  /// Liefert die MFX Tel Codierung zurück
  pub fn mfx_code(&self) -> u32 {
    match self {
      MfxCvTelBytes::Cc1byte => 0,
      MfxCvTelBytes::Cc2Byte => 1,
      MfxCvTelBytes::Cc4Byte => 2,
      MfxCvTelBytes::Cc8Byte => 3,
    }
  }
  /// Liefert MfxCvTelBytes aufgrund Anzahl Bytes
  /// # Arguments
  /// * byte_count - Anzahl Bytes (1, 2, 4 oder 8). Für alle anderen Zahlen wird None geliefert.
  fn from_count(byte_count: usize) -> Option<MfxCvTelBytes> {
    match byte_count {
      1 => Some(MfxCvTelBytes::Cc1byte),
      2 => Some(MfxCvTelBytes::Cc2Byte),
      4 => Some(MfxCvTelBytes::Cc4Byte),
      8 => Some(MfxCvTelBytes::Cc8Byte),
      _ => None,
    }
  }
}
/// Read / Write für MfxCvTel
#[derive(PartialEq)]
pub enum MfxCvTelType {
  Read,
  /// Vorbereitet für Mehrbyte Schreiben (1, 2, 4, 8), auch wenn es aktuell nicht geht, Big Endian, MSB zuerst.
  Write(Vec<u8>),
}
/// MFX CV Read/Write Telegramme senden durch DDL MFX Anfordern
pub struct MfxCvTel {
  /// Lok SID Adrdesse
  pub adr: u32,
  /// Lesen oder Schreiben?
  pub mfx_cv_type: MfxCvTelType,
  /// Anzahl Bytes
  pub byte_count: MfxCvTelBytes,
  /// CV (10 Bits)
  pub cv: u16,
  /// Index (6 Bits)
  pub index: u8,
}

/// Thread zur Ausführung MFX Dekoder Prog. Read/Write Befehlen inkl. Rückmeldungen über RDS.
/// Abarbeitung der Aufträge.
/// - Aufträge werden empfangen aus DDL Thread, MFX Protokoll
/// - Notwendige Telegramme werden zurück an DDL Thread, MFX Protokoll gesandt, da die
///   SPI Ausgabe von da aus geschehen muss.
/// - Antworten werden zurück gesendet.
///   Es erfolgt immer eine Antwort auf eine Anfrage, im Fehlerfalle "Error".
pub struct MfxRdsFeedbackThread {
  /// GPIO's zum Einlesen RDS Rückmeldung
  gpio_mfx_rds_qal: SysFsGpioInput,
  gpio_mfx_rds_clk: SysFsGpioInput,
  gpio_mfx_rds_dat: SysFsGpioInput,
  /// Receiver für Aufträge
  rx: Receiver<MfxRdsJob>,
  /// Sender für Ergenisse der Aufträge, siehe "MfxRdsJobType" als Antwort auf "ReadCA"/"WriteCA"
  tx: Sender<SmReadWrite>,
  /// Sender für Ergebnisse der Aufträge, siehe "MfxRdsJobType" als Antwort auf "ReadAllInitParameter"
  /// None: Error
  /// Some: Alle ausgelesenen Parameter (Lokname, Funktionen)
  tx_lok_init: Sender<Option<Vec<String>>>,
  /// Sender für über SPI zu versendende Telegramme
  tx_tel: Sender<MfxCvTel>,
  /// Für welche Adresse ist der aktuelle Cache gültig?
  cv_cache_adr: u32,
  /// CV Cache für "cacheAdr" (CV_Index/Value)
  cv_cache: HashMap<u16, u8>,
}

impl MfxRdsFeedbackThread {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * rx - Empfang von Aufträge.
  /// * tx - Sender zum versenden er eingelesen Rückmeldungen als Antwort auf "ReadCA"/"WriteCA"
  /// * tx_lok_init - Sender zum versenden von Lok-Init Daten als Antwort auf "ReadAllInitParameter"
  /// * tx_tel - Sender zum versenden von auszugebenden Telegrammen
  pub fn new(
    rx: Receiver<MfxRdsJob>, tx: Sender<SmReadWrite>, tx_lok_init: Sender<Option<Vec<String>>>,
    tx_tel: Sender<MfxCvTel>,
  ) -> MfxRdsFeedbackThread {
    MfxRdsFeedbackThread {
      gpio_mfx_rds_qal: SysFsGpioInput::open(GPIO_MFX_RDS_QAL)
        .expect("GPIO_MFX_RDS_QAL konnte nicht geöffnet werden"),
      gpio_mfx_rds_clk: SysFsGpioInput::open(GPIO_MFX_RDS_CLK)
        .expect("GPIO_MFX_RDS_CLK konnte nicht geöffnet werden"),
      gpio_mfx_rds_dat: SysFsGpioInput::open(GPIO_MFX_RDS_DAT)
        .expect("GPIO_MFX_RDS_DAT konnte nicht geöffnet werden"),
      rx,
      tx,
      tx_lok_init,
      tx_tel,
      cv_cache_adr: 0,
      cv_cache: HashMap::new(),
    }
  }

  /// Einlesen RDS Rückmeldung
  /// Es wird die eingelesene Rückmeldung mit verlangter Anzahl Bytes zurückgegeben.
  /// Wenn keine oder keine gültige Rückmeldung eingelesen werden konnte, wird "None" zurückgegeben.
  /// # Arguments
  /// * len - Anzahl erwarteter Bytes der Rückmeldung (1, 2, 4 oder 8)
  fn read_rds(&mut self, len: usize) -> Option<Vec<u8>> {
    //Es muss nun eine RDS Rückmeldung erfolgen.
    // - Warten bis RDS QUAL Meldung vorliegt
    // - Max. 23 mal 1, dann 010 (Startkennung)
    // - Anzahl erwartetet Datenbits
    // - 8 Bit Checksumme
    //Alles zusammen darf max. 200ms dauern, sonst wird abgebrochen
    let time_start = Instant::now();
    let mut state = StateRdsRx::StateStart1; //Warten auf QUAL und Data 1
    let mut count: usize = 0;
    //Ist ein Fehler aufgetreten? Es wird "None" als Ergebnis gesendet
    let mut result_error = false;
    //Antwort, Default "None"
    let mut result = None;
    let mut values = [0 as u8; 8];
    let mut rds_check_summe = 0 as u8;
    let mut clk_old = self.gpio_mfx_rds_clk.read_value().unwrap();
    //RDS Antwort einlesen und verarbeiten
    while state != StateRdsRx::StateFinal {
      //Warte auf nächsten Clock, positive Flanke
      loop {
        //Daten kommen etwa im 1ms Takt. Mit 100us Wartezeit sollte nichts verpasst werden
        thread::sleep(Duration::from_micros(100));
        let clk = self.gpio_mfx_rds_clk.read_value().unwrap();
        if (clk_old == GpioValue::Low) && (clk == GpioValue::High) {
          clk_old = clk;
          //Pos. Flanke erkannt -> Daten einlesen
          //Hinweis zum CLK: der verwendete RDS Chip SC6579 garantiert NICHT, ob die Daten bei pos. oder neg.
          //Flanke ändern!
          //Aber: sie ändern immer 4us vor der Flanke und sind ab einer Flanke 399us gültig.
          //Damit spielt es keine Rolle, ob pos. oder neg. Flanke verwendet wird.
          break;
        }
        clk_old = clk;
        //ggf. Abbruch wegen Timeout
        if Instant::now() > (time_start + Duration::from_millis(200)) {
          info!("MFX RDS thread Timeout.");
          result_error = true;
          break;
        }
      }
      if result_error {
        break;
      } else {
        match state {
          StateRdsRx::StateStart1 => {
            //Während Sync Sequenz sollte RDS Qual Meldung vorhanden sein.
            //Wenn nicht -> von vorne
            if self.gpio_mfx_rds_qal.read_value().unwrap() == GpioValue::Low {
              debug!("RDS Sync Abbruch. QUAL=0. 1 count={}", count);
              count = 0;
            } else {
              if self.gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                //Wieder ein 1 der Sync. Sequenz eingelesen
                count += 1;
              } else {
                //Von den 23 Bits 1 in der Sync. Sequenz will ich min. die letzten 3 gesetzt sehen, dann kann dieses 0 die Startsquenz sein
                if count >= 3 {
                  debug!("RDS new STATE_START010.");
                  state = StateRdsRx::StateStart010;
                } else {
                  //Wieder von vorne beginnen
                  debug!("RDS STATE_START1 Restart.");
                }
                count = 0;
              }
            }
          }
          StateRdsRx::StateStart010 => {
            //Erstes 0 wurde bereits gelesen, es wird noch 10 erwartet
            debug!("RDS STATE_START010 count={}.", count);
            if count == 0 {
              if self.gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                //1 gelesen, alles OK
                count += 1;
              } else {
                //0, Falsch, Abbruch
                count = 0;
                state = StateRdsRx::StateStart1;
                debug!("RDS STATE_START010 Abbruch -> STATE_START1.");
              }
            } else {
              if self.gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                //1 gelesen, Abbruch
                count = 0;
                state = StateRdsRx::StateStart1;
                debug!("RDS STATE_START010 Abbruch -> STATE_START1.");
              } else {
                //0, OK, Startsquenz ist fertig!
                count = 0;
                state = StateRdsRx::StateData;
                debug!("RDS STATE_START010 -> STATE_DATA.");
              }
            }
          }
          StateRdsRx::StateData => {
            values[count / 8] = (values[count / 8] << 1)
              | if self.gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                1
              } else {
                0
              };
            count += 1;
            if count >= (len * 8) {
              state = StateRdsRx::StateCheck;
              count = 0;
              debug!("RDS STATE_DATA -> STATE_CHECK.");
            }
          }
          StateRdsRx::StateCheck => {
            rds_check_summe = (rds_check_summe << 1)
              | if self.gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                1
              } else {
                0
              };
            count += 1;
            if count >= 8 {
              //Checksumme vollständig eingelesen
              state = StateRdsRx::StateFinal;
              debug!("RDS STATE_CHECK -> STATE_FINAL");
            }
          }
          StateRdsRx::StateFinal => {} //Nichts mehr, gesamte Schleife wird sowieso abgebrochen
        }
      }
    }
    if state == StateRdsRx::StateFinal {
      //Noch Checksumme prüfen
      let mut checksum = 0x00FF as u16;
      for i in 0..len {
        checksum ^= (checksum << 1) ^ (checksum << 2);
        checksum ^= values[i] as u16;
        if (checksum & 0x0100) != 0 {
          checksum ^= 0x0107;
        }
        if (checksum & 0x0200) != 0 {
          checksum ^= 0x020E;
        }
      }
      if checksum as u8 == rds_check_summe {
        result = Some(values[0..len].to_vec());
        info!("RDS Checksumme OK. Len={}, values={:?}", len, values);
      } else {
        warn!(
          "RDS Checksumme falsch. Len={}, values={:?}, Checksum={}, ChecksumCalc={}",
          len, values, rds_check_summe, checksum
        );
      }
    }
    result
  }

  /// Prüft, ob Cache gültig ist.
  /// Wenn für "adr" ungpltig, wird er gelöscht.
  /// # Arguments
  /// * adr - Schienenadresse des Dekoders
  fn check_cv_cache(&mut self, adr: u32) {
    if adr != self.cv_cache_adr {
      //Cache ist ungültig, neue Adresse
      self.cv_cache.clear();
      self.cv_cache_adr = adr;
    }
  }

  /// CV einer Lok abrufen.
  /// Bei einem Fehler wird bis zu 10 mal wiederholt.
  /// Liefert die ausgelesen Bytes oder None bei Fehler zurück
  /// # Arguments
  /// * adr - Schienenadresse des Dekoders
  /// * cv - Nummer des CV's (10 Bit)
  /// * index - Index im CV (6 Bit)
  /// * byteCount - Anzahl Bytes die Ab diesem CV ausgelesen werden sollen (1, 2, 4, 8)
  fn read_cv(
    &mut self, adr: u32, cv: u16, index: u8, byte_count: MfxCvTelBytes,
  ) -> Option<Vec<u8>> {
    self.check_cv_cache(adr);
    let count = byte_count.byte_count();
    //Falls im Cache, aus diesem liefern
    let cv_index = (cv << 6) | index as u16;
    let mut in_cache = true;
    let mut result = None;
    {
      for i in 0..count as u16 {
        if !self.cv_cache.contains_key(&(cv_index + i)) {
          in_cache = false;
          break;
        }
      }
    }
    if in_cache {
      let mut v: Vec<u8> = Vec::new();
      for i in 0..count as u16 {
        v.push(*self.cv_cache.get(&(cv_index + i)).unwrap());
      }
      result = Some(v);
    } else {
      //Im Fehlerfall mehrmals probieren bevor aufgegeben wird.
      for _ in 0..5 {
        self
          .tx_tel
          .send(MfxCvTel {
            adr,
            mfx_cv_type: MfxCvTelType::Read,
            byte_count: byte_count.clone(),
            cv,
            index,
          })
          .unwrap();
        //RDS Rückmeldung einlesen
        result = self.read_rds(count);
        if result.is_some() {
          //Gültige Rückmeldung in Cache aufnehmen
          let mut cv_index = cv_index;
          for val in result.clone().unwrap().iter() {
            self.cv_cache.insert(cv_index, *val);
            cv_index += 1;
          }
          break;
        }
      }
    }
    result
  }

  /// CV einer Lok schreiben.
  /// # Arguments
  /// * adr - Schienenadresse des Dekoders
  /// * cv - Nummer des CV's (10 Bit)
  /// * index - Index im CV (6 Bit)
  /// * value - Die zu schreibenden Bytes.  Vorbereitet 1, 2, 4, 8, aktuelle Dekoder unterstützen aber nur 1 Byte!
  fn write_cv(&mut self, adr: u32, cv: u16, index: u8, value: &Vec<u8>) {
    self
      .tx_tel
      .send(MfxCvTel {
        adr,
        mfx_cv_type: MfxCvTelType::Write(value.clone()),
        byte_count: MfxCvTelBytes::from_count(value.len()).unwrap(),
        cv,
        index,
      })
      .unwrap();
    //Cache löschen damit ein Lesen als verify auch tatsächlich gemacht werden muss
    self.check_cv_cache(adr);
    let cv_index = (cv << 6) | index as u16;
    for i in 0..value.len() as u16 {
      self.cv_cache.remove(&(cv_index + i));
    }
  }

  /// Findet einen bestimmten Block in den MFX CV's.
  /// Liefert None zurück wenn Block nicht gefunden wurde, sonst (start_cv, anz_gruppen, anz_ca_in_gruppe):
  /// - start_cv die Start CV Nr. des Blockes
  /// - anz_gruppen Anzahl Gruppen im Block
  /// - anz_ca_in_gruppe Anzahl CA pro Gruppe
  /// # Arguments
  /// * adr - Schienenadresse des Dekoders
  /// * block - Der gesuchte Blocktyp
  fn find_block(&mut self, adr: u32, block: BlockTypenE) -> Option<(u16, u8, u8)> {
    debug!("MFX findBlock {:?}", block);
    if block == BlockTypenE::BlockGrundeinstellungen {
      //Der erste Block mit den Dekoder Grunddaten ist immer an CV 0
      let start_cv = 0 as u16;
      //Noch Gruppen Infos auslesen, diese sind immer an Index 4
      if let Some(val) = self.read_cv(adr, 0, 4, MfxCvTelBytes::Cc2Byte) {
        let anz_gruppen = val[0];
        let anz_ca_in_gruppe = val[1];
        return Some((start_cv, anz_gruppen, anz_ca_in_gruppe));
      } else {
        //Fehler, Abbruch
        warn!("MFX findBlock Abbruch. readCV fail. SID={}", adr);
        return None;
      }
    }
    //Alle andern Blöcke müssen gesucht werden -> zuerst Liste mit allen Blöcken auslesen
    //Das gibt eine kleine Rekursion...
    let block_as_u8 = block.clone() as u8;
    if let Some((_cv_bl, block_liste)) = self.read_ca(
      adr,
      BlockTypenE::BlockGrundeinstellungen,
      BlockCaE::CaGrundBlocktab,
      0,
    ) {
      //Blockliste abarbeiten
      for i in 0..block_liste.len() {
        if block_liste[i] == 0 {
          //Block nicht gefunden, keiner mehr vorhanden
          warn!(
            "MFX findBlock Abbruch. Block nicht gefunden. SID={} Block={:?}",
            adr, block
          );
          return None;
        }
        let start_cv = block_liste[i] as u16 * 4;
        debug!("MFX Blockliste Index={}, Block at CV={}", i, start_cv);
        if let Some(block_id) = self.read_cv(adr, start_cv, 1, MfxCvTelBytes::Cc1byte) {
          if block_id[0] == block_as_u8 {
            debug!("Block {:?} gefunden an CV={}", block, start_cv);
            //Block gefunden
            //Noch Gruppen Infos auslesen
            if let Some(block_groesse) = self.read_cv(adr, start_cv, 4, MfxCvTelBytes::Cc2Byte) {
              let anz_gruppen = block_groesse[0];
              let anz_cain_gruppe = block_groesse[1];
              return Some((start_cv, anz_gruppen, anz_cain_gruppe));
            } else {
              //Fehler, Abbruch
              warn!("findBlock Abbruch. readCV fail. SID={}", adr);
              return None;
            }
          }
        } else {
          //Fehler, Abbruch
          warn!("findBlock Abbruch. readCV fail. SID={}", adr);
          return None;
        }
      }
    } else {
      //Fehler, Abbruch
      warn!("MFX findBlock Abbruch. readCA fail. SID={}", adr);
      return None;
    }
    None
  }

  /// Liest eine gewünschte CA aus.
  /// Liefert None bei einem Fehler, sonst (cv, values):
  /// - cv : CV an dem CA gefunden wurde
  /// - values : alle gelesenen Bytes. Da immer 4 Byte auf einmal gelesen werden also 4, 8, 12, .. Bytes
  /// # Arguments
  /// * adr - Schienenadresse des Dekoders
  /// * block - Blocktyp in dem nach der CA gesucht wird.
  /// * ca - gesuchte CA im Block
  /// * caIndex - Das wievielte Vorkommen des CA's wird gesucht? Erstes Vorkommen ist die 0
  fn read_ca(
    &mut self, adr: u32, block: BlockTypenE, ca: BlockCaE, ca_index: u8,
  ) -> Option<(u16, Vec<u8>)> {
    debug!(
      "MFX readCA adresse={}, block={:?}, ca={:?}\n",
      adr, block, ca
    );
    let (ca_len, ca_id) = ca.value();
    if let Some(cv) = self.find_ca(adr, block.clone() as u8, ca_id, ca_index) {
      //CV zu CA gefunden
      debug!("CA {:?} gefunden an CV {}", ca, cv);
      //Ganzer CA auslesen
      let mut result: Vec<u8> = Vec::new();
      //Eigentlich immer > 0, nur CaNichtVerwendet hat len=0
      if ca_len > 0 {
        for i in 0..=((ca_len - 1) / 4) {
          //Start ab Index 1 (nach CA Typ)
          if let Some(val) = self.read_cv(adr, cv, 1 + (i * 4), MfxCvTelBytes::Cc4Byte) {
            result.extend_from_slice(val.as_slice());
          } else {
            //Fehler, Abbruch
            warn!("MFX readCA Abbruch. readCV fail. SID={}, CV={}", adr, cv);
            return None;
          }
        }
      }
      return Some((cv, result));
    } else {
      warn!(
        "SID={}, CA={:?} in Block {:?} nicht gefunden\n",
        adr, ca, block
      );
      return None;
    }
  }

  /// Findet eine gewünschte CA in einem gewünschten Block.
  /// Liefert gefundene CV Adresse zurück, None wenn nicht gefunden
  /// # Arguments
  /// * adr - Schienenadresse des Dekoders
  /// * block - Blocktyp in dem nach der CA gesucht wird.
  /// * ca - gesuchte CA im Block
  /// * ca_index - Das wievielte Vorkommen des CA's wird gesucht? Erstes Vorkommen ist die 0
  fn find_ca(&mut self, adr: u32, block: u8, ca: u8, mut ca_index: u8) -> Option<u16> {
    debug!(
      "MFX findCA adresse={}, block={}, ca={}, caIndex={}",
      adr, block, ca, ca_index
    );
    if let Some(bl) = BlockTypenE::from(block) {
      if let Some((mut cv, anz_gruppen, anz_cain_gruppe)) = self.find_block(adr, bl) {
        //Alle CA's in diesem Block durchsuchen
        for _ in 0..=(anz_gruppen * anz_cain_gruppe) {
          //..= um auch den ersten CA mit Blockbeschreibung zu berücksichtigen
          if let Some(ca_typ) = self.read_cv(adr, cv, 0, MfxCvTelBytes::Cc1byte) {
            //Ist das der gesuchte CA?
            if ca_typ[0] == ca {
              debug!("MFX CA {:?} gefunden an CV {}", ca, cv);
              if ca_index == 0 {
                //Gewünschter CA gefunden
                return Some(cv);
              }
              ca_index -= 1;
            }
            cv += 1;
          } else {
            //Fehler, Abbruch
            warn!("MFX findCA Abbruch. readCV fail. SID={}, CV={}", adr, cv);
            return None;
          }
        }
      } else {
        warn!("MFX findCA findBlock fail. SID={}, Block={:?}", adr, block);
        return None;
      }
    } else {
      warn!("MFX findCA ungültiger Block. SID={}, Block={}", adr, block);
      return None;
    }
    //Fehler, CA nicht gefunden
    warn!(
      "MFX SID={}, CA={} in Block {} nicht gefunden",
      adr, ca, block
    );
    None
  }

  /// Name und Funktionen einer Lok lesen.
  /// Liefert None zurück wenn ein Fehler aufgetreten ist, sonst Name und die ersten 16 Funktionen.
  /// Jede Funktion 32 Bit, jedoch nur die 3 Unterbytes verwendet (Funktionsgruppe, Symbolinfo 1 und 2)
  /// # Arguments
  /// * adresse - Schienenadresse des Dekoders
  fn read_lok_name_fx(&mut self, adr: u32) -> Option<(String, [u32; MFX_FX_COUNT])> {
    //Lokname
    let name: String;
    if let Some((_cv, name_bin)) = self.read_ca(
      adr,
      BlockTypenE::BlockGrundeinstellungen,
      BlockCaE::CaGrundLokname,
      0,
    ) {
      if let Ok(str) = String::from_utf8(name_bin) {
        //Spaces am Schluss abschneiden falls vorhanden
        name = str.trim().to_string();
        debug!("MFX SID={} Lokname: {}", adr, name);
      } else {
        warn!("MFX SID={} Lokname ist ungültig!", adr);
        name = String::from("?");
      }
    } else {
      //Fehler, Abbruch
      warn!(
        "MFX readLokNameFx Abbruch. readCA {:?} fail. SID={}",
        BlockCaE::CaGrundLokname,
        adr
      );
      return None;
    }

    //Funktionszuordnungen
    //Auslesen Funktionsindexe für Funktionsdefinitionen im Block BLOCK_FUNKTION_MAPPING
    if let Some((_cv, funktionen)) = self.read_ca(
      adr,
      BlockTypenE::BlockFunktionalitaet,
      BlockCaE::CaFunkSchaltfunktion,
      0,
    ) {
      //Funktionstypen auslesen
      //Zuerst Start CV des Block BLOCK_FUNKTION_MAPPING ermitteln (in zurückgelesene Blockinfodaten werden nicht benötigt)
      if let Some((cv, _blockinfo)) = self.read_ca(
        adr,
        BlockTypenE::BlockFunktionMapping,
        BlockCaE::CaBlockBeschreibung,
        0,
      ) {
        //Für alle Funktionen die Funktionszuordnungen
        let mut fx: [u32; MFX_FX_COUNT] = [0; MFX_FX_COUNT];
        //Nun haben wir in cv die CV Nummer, an der der BLOCK_FUNKTION_MAPPING startet
        for i in 0..min(funktionen.len(), fx.len()) {
          if let Some(funktion) =
            self.read_cv(adr, cv + funktionen[i] as u16, 0, MfxCvTelBytes::Cc4Byte)
          {
            fx[i] = ((funktion[1] as u32) << 16) | ((funktion[2] as u32) << 8) | funktion[3] as u32;
            debug!(
              "F{} {} Gruppe=0{} S1=0{} S2={}",
              i,
              if (funktion[1] & 0x80) != 0 { 'I' } else { 'D' },
              funktion[1] & 0x7F,
              funktion[2],
              funktion[3]
            );
          } else {
            //Fehler, Abbruch
            warn!(
              "MFX readLokNameFx Abbruch. read_cv cv={} fail. SID={}",
              cv + funktionen[i] as u16,
              adr
            );
            return None;
          }
        }
        //Name und Funktionen konnten ausgelesen werden!
        return Some((name, fx));
      } else {
        //Fehler, Abbruch
        warn!(
          "MFX readLokNameFx Abbruch. readCA {:?} fail. SID={}",
          BlockCaE::CaBlockBeschreibung,
          adr
        );
        return None;
      }
    } else {
      //Fehler, Abbruch
      warn!(
        "MFX readLokNameFx Abbruch. readCA {:?} fail. SID={}",
        BlockCaE::CaFunkSchaltfunktion,
        adr
      );
      return None;
    }
  }

  /// Als Thread ausführen
  pub fn execute(&mut self) -> ! {
    loop {
      //Warten auf Arbeit
      let auftrag = self.rx.recv().unwrap();
      match auftrag {
        MfxRdsJob::ReadAllInitParameter { adr } => {
          //Lokname und Funktionen lesen
          if let Some((name, fx)) = self.read_lok_name_fx(adr) {
            //Alle Init Parameter als String, Lokname kommt in Anführungszeichen
            let mut para: Vec<String> = Vec::new();
            para.push(format!("\"{}\"", name.as_str()));
            for i in 0..fx.len() {
              para.push(fx[i].to_string());
            }
            self.tx_lok_init.send(Some(para)).unwrap();
          } else {
            warn!(
              "MFX Lokname und Funktionen konnten nicht gelesen werden. SID={}",
              adr
            );
            self.tx_lok_init.send(None).unwrap();
          }
        }
        MfxRdsJob::ReadWriteCA { mut ca_parameter } => {
          let block = ca_parameter.para[0] as u8;
          let ca = ca_parameter.para[1] as u8;
          let ca_index = ca_parameter.para[2] as u8;
          let index = ca_parameter.para[3] as u8;
          if let Some(cv) = self.find_ca(ca_parameter.adr, block, ca, ca_index) {
            match ca_parameter.val {
              SmReadWriteType::Read => {
                //Read
                if let Some(val) = self.read_cv(ca_parameter.adr, cv, index, MfxCvTelBytes::Cc1byte)
                {
                  //Alles OK, gelesener Wert als Antwort zurück senden
                  ca_parameter.val = SmReadWriteType::ResultOk(val[0] as u32);
                } else {
                  warn!(
                    "MFX Error ReadCA read_cv {}.{} für SID={}",
                    cv, index, ca_parameter.adr
                  );
                  ca_parameter.val = SmReadWriteType::ResultErr;
                }
              }
              SmReadWriteType::Write(val) => {
                //Write
                self.write_cv(ca_parameter.adr, cv, index, &vec![val as u8]);
              }
              SmReadWriteType::Verify(val_ver) => {
                //Zuerst Read
                if let Some(val) = self.read_cv(ca_parameter.adr, cv, index, MfxCvTelBytes::Cc1byte)
                {
                  //Alles OK, gelesener Wert vergleichen
                  ca_parameter.val = if val_ver == val[0] as u32 {
                    SmReadWriteType::ResultOk(val[0] as u32)
                  } else {
                    SmReadWriteType::ResultErr
                  };
                } else {
                  warn!(
                    "MFX Error ReadCA read_cv {}.{} für SID={}",
                    cv, index, ca_parameter.adr
                  );
                  ca_parameter.val = SmReadWriteType::ResultErr;
                }
              }
              _ => {
                //Keine Aktion Results
                warn!(
                  "MFX Error nicht unterstützter Kommandotype {:?}",
                  ca_parameter.val
                );
                ca_parameter.val = SmReadWriteType::ResultErr;
              }
            }
          } else {
            warn!("MFX Error ReadCA findCA {:?}", ca_parameter,);
            ca_parameter.val = SmReadWriteType::ResultErr;
          }
          //Antwort zurück senden, OK wenn ca_parameter.val vorhanden, sonst Error
          self.tx.send(ca_parameter).unwrap();
        }
      }
    }
  }
}

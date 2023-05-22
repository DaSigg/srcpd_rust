use std::time::Duration;

use crate::srcp_protocol_ddl::{DdlProtokoll, DdlTel, GLDriveMode};

/// SPI Baudrate für Märklin / Motorola Protokoll.
/// Diese wäre eigentlich genau 38461 Baud (1 Bit=26us, 1Byte=208us)
const SPI_BAUDRATE_MAERKLIN_LOCO: u32 = 38461;
/// Nun macht der RaspberryPI aber leider nach jedem Byte Transfer auf dem SPI Bus 1 Bit Pause :-(
/// Er macht diese 1 Bit / Clk Pause nicht mehr im DMA Mode. DMA wird ab Transfers
/// von 96 Bytes verwendet (hoffe, das bleibt so ...).
/// Um Märklin / Motorola Pakete auf 96 rauf zu bringen, wird folgendes gemacht:
/// - die Baudrate Doppelt so gewählt wie notwendig (2 * 38461 für Loks)
/// - die Wiederholung ins selbe Paket gepackt.
/// - Pause am Anfang und vor Wiederholung mit 0 Bytes gefüllt.
pub const SPI_BAUDRATE_MAERKLIN_LOCO_2: u32 = 2 * SPI_BAUDRATE_MAERKLIN_LOCO;
const SPI_BAUDRATE_MAERKLIN_FUNC_2: u32 = 2 * SPI_BAUDRATE_MAERKLIN_LOCO_2;
/// Für Märklin Motorola wird wie folgt kodiert (doppelte Baudrate):
/// - Paket mit
///  - 0 -> 0xC0, 0x00, ich habe aber Schaltdekoder, die damit nicht funktionieren sondern einen ein wenig längeren Impuls wollen, also 0xE0, 0x00 ....
///  - 1 -> 0xFF, 0xFC
///  -> 18 * 2 = 36 Bytes
const MM_LEN_PAKET: usize = 18 * 2;
/// - 0 Bytes für Pause zwischen Paketen: 3 * (t 2 Bit, 208us / 416us) -> wegen doppelter Baudrate also 2 * 3 * 2 = 12 Bytes 0x00
const MM_LEN_PAUSE_BETWEEN: usize = 2 * 3 * 2;
/// - Paket Wiederholung
/// - 0 Bytes für Pause nach Paket: 4.2ms (Lok), resp. 2.1ms (Schaltdekoder) -> wegen bei doppleter Baudrate 1 Byte 104us (Lok). 62us (Schalt) = 42 Bytes
const MM_LEN_PAUSE_END: usize = 42;
/// Pause am Anfang/Ende (Anfang nur, falls vorher keine Pause war)
const MM_PAUSE_GA: Duration = Duration::from_micros(2100);
const MM_PAUSE_GL: Duration = Duration::from_micros(4200);
/// Total also 36 + 12 + 36 + 42 = 126 Bytes -> DMA Mode!
const MM_LEN: usize = MM_LEN_PAKET + MM_LEN_PAUSE_BETWEEN + MM_LEN_PAKET + MM_LEN_PAUSE_END;
/// Mit doppelter Baudrate je die beiden Bytes für 0 und 1 Übertragung
const MM_BIT_0_0: u8 = 0xC0;
const MM_BIT_0_0_GA: u8 = 0xE0; //Eigentlich wäre das obige 0xC0 korrekt, habe aber Schaltdekoder die damit nicht funktionieren....
const MM_BIT_0_1: u8 = 0x00;
const MM_BIT_1_0: u8 = 0xFF;
const MM_BIT_1_1: u8 = 0xFC;
static MM_BIT_0: &'static [u8] = &[MM_BIT_0_0, MM_BIT_0_1]; //0
static MM_BIT_1: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1]; //1
static MM_BIT_L: &'static [u8] = &[MM_BIT_0_0, MM_BIT_0_1, MM_BIT_0_0, MM_BIT_0_1]; //00
static MM_BIT_H: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1, MM_BIT_1_0, MM_BIT_1_1]; //11
static MM_BIT_O: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1, MM_BIT_0_0, MM_BIT_0_1]; //10
static MM_BIT_L_GA: &'static [u8] = &[MM_BIT_0_0_GA, MM_BIT_0_1, MM_BIT_0_0_GA, MM_BIT_0_1]; //00 für GA, siehe oben
static MM_BIT_O_GA: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1, MM_BIT_0_0_GA, MM_BIT_0_1]; //10 für GA, siehe oben
static MM_BIT_U: &'static [u8] = &[MM_BIT_0_0, MM_BIT_0_1, MM_BIT_1_0, MM_BIT_1_1]; //01

/// MM2 & 3 Bitmuster für F1-4, Bit 3 ist jeweils der Zustand der Funktion
static MM_F1_4: &'static [u8] = &[0b0011, 0b0100, 0b0110, 0b0111];

/// Pause zwischen zwei Speed Paketen für MM5 (nur für MM5 relevant)
const MM_PAUSE_MM5: Duration = Duration::from_millis(50);
/// Max. erlaubte Dekoder Adresse (GA und GL)
const MAX_MM_ADRESSE: usize = 80;
/// Max. erlaubte GA Adresse (4 GA per Dekoder)
const MAX_MM_GA_ADRESSE: usize = (MAX_MM_ADRESSE + 1) * 4;
/// Implementierung Märklin Motorola Protokoll V1 & 2
#[derive(PartialEq, Copy, Clone)]
pub enum MmVersion {
  V1, //14 v Stufen, F0, relative Richtung
  V2, //14 v Stufen, F0-4, absolute Richtung
  V3, //28 v Stufen mit Halbstufe über 2. Bit F0, F0-4, absolute Richtung
  V5, //28 v Stufen über Senden 2 benachbarte Stufen kurz nacheinander, F0-4, absolute Richtung
}
pub struct MMProtokoll {
  /// Version 1 oder 2, Keine Unterschiede für GA, nur für GL 14 / 28 Fahrstufen, 1 oder 5 Funktionen
  version: MmVersion,
  /// Erkennung Richtungswechsel bei M1, Halten Richtung bei Richtung Nothalt bei M1 und M2
  old_drive_mode: [GLDriveMode; MAX_MM_ADRESSE + 1],
  /// Erkennung Funktionswechsel bei M2 & 3
  old_funktionen: [u64; MAX_MM_ADRESSE + 1],
  /// Speicherung Speed um F1-F4 Pakete für MM2 & 3, die auch den Speed enthalten, korrekt erzeugen zu können
  old_speed: [usize; MAX_MM_ADRESSE + 1],
  /// Anzahl Initialisierte Funktionen
  funk_anz: [usize; MAX_MM_ADRESSE + 1],
}
impl MMProtokoll {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * version - V1 oder V2
  pub fn from(version: MmVersion) -> MMProtokoll {
    MMProtokoll {
      version,
      old_drive_mode: [GLDriveMode::Vorwaerts; MAX_MM_ADRESSE + 1],
      old_funktionen: [0; MAX_MM_ADRESSE + 1],
      old_speed: [0; MAX_MM_ADRESSE + 1],
      funk_anz: [0; MAX_MM_ADRESSE + 1],
    }
  }
  /// MM 4 Adressbits (trinär codiert)
  /// Adresse 80 wird als 0000 gesendet, die eigentliche Adresse 80 ist der Idlestate, Lok 0 gibt es nicht
  /// # Arguments
  /// * ddl_tel - Telegramm, zu dessen letztem Telegramm die Adressbits hinzugefügtw erden sollen
  /// * adr_dekoder - Adresse, die ergänzt werden soll, LSB wird zuerst gesendet, 0..80 erlaubt.
  /// * ga_timing - Impulsverbreituerung 0 für GA's, siehe Kommentar zu MM_BIT_0_0_GA
  fn add_mm_adr(&self, ddl_tel: &mut DdlTel, mut adr_dekoder: usize, ga_timing: bool) {
    assert!(adr_dekoder < 81, "MM Max Lokadresse ist 80");
    if adr_dekoder == 80 {
      adr_dekoder = 0;
    }
    let mm_bit_l = if ga_timing { MM_BIT_L_GA } else { MM_BIT_L };
    let mm_bit_o = if ga_timing { MM_BIT_O_GA } else { MM_BIT_O };
    for _ in 0..4 {
      let adr_trit = adr_dekoder % 3;
      adr_dekoder /= 3;
      match adr_trit {
        0 => {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(mm_bit_l);
        }
        1 => {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(MM_BIT_H);
        }
        2 => {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(mm_bit_o);
        }
        _ => assert!(false), //Kann nicht vorkommen da Rest der Division mit 3
      }
    }
  }
  /// MM1 Payload 5 Bits, 1 Bit Funktion und 4 Bits Value
  /// # Arguments
  /// * ddl_tel - Telegramm, zu dessem letzten Tel. die Adressbits hinzugefügtw erden sollen
  /// * fnkt - true: Funktionsbit 1, false für 0
  /// * value - 4 Bit Value, LSB wird zuerst gesendet
  /// * ga_timing - Impulsverbreituerung 0 für GA's, siehe Kommentar zu MM_BIT_0_0_GA
  fn add_mm1_fnkt_value(
    &self, ddl_tel: &mut DdlTel, fnkt: bool, mut value: usize, ga_timing: bool,
  ) {
    let mm_bit_l = if ga_timing { MM_BIT_L_GA } else { MM_BIT_L };
    //Zuerst kommt die Funktion
    if fnkt {
      ddl_tel
        .daten
        .last_mut()
        .unwrap()
        .extend_from_slice(MM_BIT_H);
    } else {
      ddl_tel
        .daten
        .last_mut()
        .unwrap()
        .extend_from_slice(mm_bit_l);
    }
    //Dann Value, 4 Bit, LSB als erstes
    assert!(value <= 0x0F);
    for _ in 0..4 {
      if (value & 0x01) == 0 {
        ddl_tel
          .daten
          .last_mut()
          .unwrap()
          .extend_from_slice(mm_bit_l);
      } else {
        ddl_tel
          .daten
          .last_mut()
          .unwrap()
          .extend_from_slice(MM_BIT_H);
      }
      value >>= 1;
    }
  }
  /// MM2 Payload 5 Bits, 1 Bit Funktion und 4 Bits Speed
  /// # Arguments
  /// * ddl_tel - Telegramm, zu dessen letztem Tel. die Adressbits hinzugefügtw erden sollen
  /// * fnkt - Funktionsbit MM1&2:  oder MM_BIT_L, MM3 antivalent für Speed-Halfstep, erstes Bit Funktion
  /// * speed - 4 Bit Value, LSB wird zuerst gesendet
  /// * dir - Fahrtrichtung, Rückwärts wird ausgewertet, alles andere ist Vorwärts
  fn add_mm2_fnkt_value(
    &self, ddl_tel: &mut DdlTel, fnkt: &[u8], mut speed: usize, dir: GLDriveMode,
  ) {
    //Zuerst kommt die Funktion
    ddl_tel.daten.last_mut().unwrap().extend_from_slice(fnkt);
    //Bei MM2 wird nur je ein Bit des Paares für die Geschwindigkeit verwendet,
    //das andere für die absolute Richtungsinfo.
    //Parallel zum Speed kodieren wir hier die abs. Richtungsbits
    let mut abs_dir: usize;
    if dir == GLDriveMode::Rueckwaerts {
      //Rückwärts
      if speed <= 7 {
        abs_dir = 0b1101;
      } else {
        abs_dir = 0b0101;
      }
    } else {
      //Vorwärts
      if speed <= 7 {
        abs_dir = 0b1010;
      } else {
        abs_dir = 0b0010;
      }
    }
    //Dann Speed, 4 Bit, LSB als erstes, Verknüpft mit abs. Richtung
    assert!(speed <= 15);
    for _ in 0..4 {
      if (speed & 0x01) == 0 {
        if (abs_dir & 0x01) == 0 {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(MM_BIT_L);
        } else {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(MM_BIT_U);
        }
      } else {
        if (abs_dir & 0x01) == 0 {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(MM_BIT_O);
        } else {
          ddl_tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(MM_BIT_H);
        }
      }
      speed >>= 1;
      abs_dir >>= 1;
    }
  }
  /// Erzeugt das Basis Telegramm für GL ohne Wiederholung und Pausen.
  /// - Fahren
  /// - Funktionen F0
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * drive_mode - Fahrtrichtung / Nothalt
  /// * speed - aktuelle Geschwindigkeit
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis F0
  /// * ddl_tel - Telegramm in dessen letzten Tel. das Basis Tel. erzeugt werden soll.
  /// * version - Für welche MM Version.
  fn get_gl_basis_tel_raw(
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, funktionen: u64,
    ddl_tel: &mut DdlTel, version: MmVersion,
  ) {
    let mut drive_mode_used = drive_mode;
    let mut speed_used = speed;
    if drive_mode == GLDriveMode::Nothalt {
      //Nothalt gibt es nicht -> kein Richtungswechsel, Speed = 0
      drive_mode_used = self.old_drive_mode[adr];
      speed_used = 0;
    }
    if speed_used > 0 {
      speed_used += 1; //Speed 1 ist Richtungswechsel, mit Speed 1..14 sind wir damit bei 2..15, mit 1..28 bei 2..29
    }
    self.add_mm_adr(ddl_tel, adr, false);
    match version {
      MmVersion::V1 => {
        //14 Speeds, F0, rel. Richtung
        //Max Speed Kontrolle
        if speed_used > 15 {
          speed_used = 15;
        }
        //Richtungswechsel
        if drive_mode_used != self.old_drive_mode[adr] {
          speed_used = 1;
        }
        self.add_mm1_fnkt_value(
          ddl_tel,
          (funktionen & 0x01) != 0, //F0
          speed_used,
          false,
        );
      }
      MmVersion::V2 => {
        //14 Speeds, F0-4, abs. Richtung
        //Max Speed Kontrolle
        if speed_used > 15 {
          speed_used = 15;
        }
        //Fahren mit abs. Richtung
        self.add_mm2_fnkt_value(
          ddl_tel,
          if (funktionen & 0x01) != 0 {
            MM_BIT_H
          } else {
            MM_BIT_L
          }, //F0
          speed_used,
          drive_mode_used,
        );
      }
      MmVersion::V3 => {
        //28 Speeds, Halbschritt über 2. Bit F0, F0-4, abs. Richtung, analog V2, zusätzlicher Speed Schritt über 2. Bit F0
        //Max Speed Kontrolle
        if speed_used > 28 {
          speed_used = 28;
        }
        let speed_halfstep = (speed_used % 2) != 0;
        //Nun Speed auf den Range 0..15 wie für V1,2 zurück skalieren
        //0 bleibt 0, 2..29 wird 2..15
        let speed = (speed_used / 2) + if speed_used > 0 { 1 } else { 0 };
        //Fahren mit abs. Richtung
        self.add_mm2_fnkt_value(
          ddl_tel,
          //F0 mit Speed Halfstep
          if (funktionen & 0x01) != 0 {
            if speed_halfstep {
              MM_BIT_O
            } else {
              MM_BIT_H
            }
          } else {
            if speed_halfstep {
              MM_BIT_U
            } else {
              MM_BIT_L
            }
          },
          speed,
          drive_mode_used,
        );
      }
      MmVersion::V5 => {
        //28 Speeds mittels Senden 2 benachbarte Steps nacheinander, F0-4, abs. Richtung, analog V2
        //Max Speed Kontrolle
        if speed_used > 28 {
          speed_used = 28;
        }
        /* xFS    rFS   sFS1 -> (50ms)  -> sFS2
          0      0      0                  %

          2      2      0                  2
          3    2.5      3                  2
          4      3      2                  3
          5    3.5      4                  3
          6      4      3                  4
          7    4.5      5                  4
          8      5      4                  5
          9    5.5      6                  5
          10     6      5                  6
          11   6.5      7                  6
          12     7      6                  7
          13   7.5      8                  7
          14     8      7                  8
          15   8.5      9                  8
          16     9      8                  9
          17   9.5     10                  9
          18    10      9                 10
          19  10.5     11                 10
          20    11     10                 11
          21  11.5     12                 11
          22    12     11                 12
          23  12.5     13                 12
          24    13     12                 13
          25  13.5     14                 13
          26    14     13                 14
          27  14.5     15                 14
          28    15     14                 15
        */
        let speed_full_step = if speed_used == 0 {
          0
        } else {
          (speed_used / 2) + 1
        }; //Ergibt 0, 2..15
        let speed_half_step = match speed_used {
          0 => None,
          2 => Some(0),
          _ => Some(if (speed_used % 2) == 0 {
            speed_full_step - 1
          } else {
            speed_full_step + 1
          }),
        };
        //Fahren mit abs. Richtung
        if let Some(speed_half) = speed_half_step {
          //Zwischen diesen Telegrammen muss eine 50ms Pause liegen
          ddl_tel.delay = MM_PAUSE_MM5;
          self.add_mm2_fnkt_value(
            ddl_tel,
            if (funktionen & 0x01) != 0 {
              MM_BIT_H
            } else {
              MM_BIT_L
            }, //F0
            speed_half,
            drive_mode_used,
          );
          //2. Telegramm vorbereiten
          ddl_tel.daten.push(Vec::with_capacity(MM_LEN));
          self.add_mm_adr(ddl_tel, adr, false);
        }
        self.add_mm2_fnkt_value(
          ddl_tel,
          if (funktionen & 0x01) != 0 {
            MM_BIT_H
          } else {
            MM_BIT_L
          }, //F0
          speed_full_step,
          drive_mode_used,
        );
      }
    }
    self.old_drive_mode[adr] = drive_mode_used;
    self.old_speed[adr] = speed_used;
    //und noch F0 übernehmen
    self.old_funktionen[adr] &= !1; //F0 löschen
    self.old_funktionen[adr] |= funktionen & 1; //und neu übernehmen löschen
  }

  /// MM Paket vervollständigen (für alle telegramme, falls mehrere vorhanden sind):
  /// - Pause zwischen den beiden Paketen
  /// - Paketwiederholung
  /// - Pause am Schluss
  fn complete_mm_paket(&self, ddl_tel: &mut DdlTel) {
    let ddl_daten = ddl_tel.daten.last_mut().unwrap();
    //Pause zwischen den beiden Paketen ergänzen
    ddl_daten.resize(ddl_daten.len() + MM_LEN_PAUSE_BETWEEN, 0);
    //Wiederholung
    let tel: Vec<u8> = ddl_daten[0..MM_LEN_PAKET].to_vec();
    ddl_daten.extend(&tel);
    //Pause am Schluss
    ddl_daten.resize(ddl_daten.len() + MM_LEN_PAUSE_END, 0);
  }
}
impl DdlProtokoll for MMProtokoll {
  /// GL Init Daten setzen. Welche Daten verwendet werden ist Protokollabhängig.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * uid - UID des Dekoders -> hier nicht verwendet
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  fn init_gl(&mut self, adr: usize, _uid: u32, funk_anz: usize) {
    self.funk_anz[adr] = funk_anz;
  }
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> usize {
    MAX_MM_ADRESSE
  }
  /// Liefert die max. erlaubte Schaltmoduladdresse
  fn get_ga_max_adr(&self) -> usize {
    MAX_MM_GA_ADRESSE
  }
  /// Liefert die max. Anzahl der unterstützten Funktionen
  fn get_gl_anz_f(&self) -> usize {
    match self.version {
      MmVersion::V1 => 1, //V1 nur F0
      MmVersion::V2 => 5, //V2 F0-F4
      MmVersion::V3 => 5, //V3 F0-F4
      MmVersion::V5 => 5, //V5 F0-F4
    }
  }
  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize {
    //Es ist nur F0 im Tel. mit Speed und Richtung vorhanden
    1
  }
  /// Liefert ein leeres GL Telegramm zur Verwendung in "get_gl_basis_tel" und / oder "get_gl_zusatz_tel".
  /// # Arguments
  /// * adr - Adresse der Lok, keine Verwendunbg, nur Debug Support
  fn get_gl_new_tel(&self, adr: usize) -> DdlTel {
    DdlTel::new(
      adr,
      SPI_BAUDRATE_MAERKLIN_LOCO_2,
      MM_PAUSE_GL,
      MM_PAUSE_GL,
      Duration::ZERO,
      MM_LEN,
    )
  }

  /// Erzeugt das Basis Telegramm für GL.
  /// - Fahren
  /// - Basisfunktionen F0 bis "get_Anz_F_Basis". Es wedren hier nur diese Funktionen übernommen!
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * drive_mode - Fahrtrichtung / Nothalt
  /// * speed - aktuelle Geschwindigkeit
  /// * speed_steps - Anzahl Speed Steps die verwendet werden soll. Protokoll abhängig.
  ///                 Hier nicht verwendet da durch Protokollversion gegeben.
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_basis_tel(
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, _speed_steps: usize,
    funktionen: u64, ddl_tel: &mut DdlTel,
  ) {
    self.get_gl_basis_tel_raw(adr, drive_mode, speed, funktionen, ddl_tel, self.version);
    self.complete_mm_paket(ddl_tel);
  }
  /// Erzeugt das / die Fx Zusatztelegramm(e) für GL.
  /// - Funktionen nach "get_Anz_F_Basis"
  /// Alle notwendigen Telegramme werden einzeln zu "ddl_tel" hinzugefügt.
  /// Wenn keine extra Telegramme für Zusatzfunktionen vorhanden sind, dann wird nichts hinzugefügt.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * refresh - Wenn false werden nur Telegramme für Funktionen, die geändert haben, erzeugt
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt ab "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_zusatz_tel(
    &mut self, adr: usize, refresh: bool, funktionen: u64, ddl_tel: &mut DdlTel,
  ) {
    if self.version == MmVersion::V1 {
      return;
    }
    let funk_anz = self.funk_anz[adr];
    //Nun noch F1-4, jedoch nur bei Veränderung sofort senden
    for i in 1..self.get_gl_anz_f() {
      if i >= funk_anz {
        break;
      }
      let mask: u64 = 1 << i;
      if (((self.old_funktionen[adr] ^ funktionen) & mask) != 0) || refresh {
        //Veränderung oder immer verlangt
        //Neues Telegramm erzeugen
        ddl_tel.daten.push(Vec::with_capacity(MM_LEN));
        //Als Basis Standard Fahren Telegramm verwenden und dieses dann auf F1-4 ändern
        self.get_gl_basis_tel_raw(
          adr,
          self.old_drive_mode[adr],
          self.old_speed[adr],
          funktionen,
          ddl_tel,
          MmVersion::V2, //Hier immer V2, keine 1/2 Speed Steps
        );
        let mut fx_bits = MM_F1_4[i - 1];
        //Zustand der Funktion ergänzen
        if (funktionen & mask) != 0 {
          fx_bits |= 0b1000;
        }
        for bit in 0..4 {
          //Bit 11 13 15. Da wegen doppelter Baurate 2 Byte pro Bit nochmals * 2
          let faktor_baudrate = MM_BIT_0.len();
          for j in 0..faktor_baudrate {
            ddl_tel.daten.last_mut().unwrap()[faktor_baudrate * (11 + bit * 2) + j] =
              if (fx_bits & 0b0001) == 0 {
                MM_BIT_0[j]
              } else {
                MM_BIT_1[j]
              };
          }
          fx_bits >>= 1;
        }
        self.complete_mm_paket(ddl_tel);
      }
    }
    self.old_funktionen[adr] = funktionen;
  }
  /// Liefert ein leeres GA Telegramm zur Verwendung in "get_ga_tel".
  /// # Arguments
  /// * adr - Adresse GA, keine Verwendunbg, nur Debug Support
  fn get_ga_new_tel(&self, adr: usize) -> DdlTel {
    DdlTel::new(
      adr,
      SPI_BAUDRATE_MAERKLIN_FUNC_2,
      MM_PAUSE_GA,
      MM_PAUSE_GA,
      Duration::ZERO,
      MM_LEN,
    )
  }
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_ga_tel(&self, adr: usize, port: usize, value: bool, ddl_tel: &mut DdlTel) {
    //Dekoderadresse: 4 Ausgangspaare auf Dekoder, deshalb adr/4
    let adr_dekoder = (adr - 1) >> 2;
    //Subadresse auf Dekoder ist welches der 4 Paare plus Port
    let sub_adr = (((adr - 1) & 3) << 1) + (port & 1);
    self.add_mm_adr(ddl_tel, adr_dekoder, true);
    self.add_mm1_fnkt_value(
      ddl_tel,
      false,
      sub_adr + (if value { 0x08 } else { 0x00 }), //Value ist das 4. Bit
      true,
    );
    self.complete_mm_paket(ddl_tel);
  }

  /// Liefert das Idle Telegramm dieses Protokolles
  /// Return None wenn kein Idle Telegramm vorhanden ist
  fn get_idle_tel(&mut self) -> Option<DdlTel> {
    //Idle Telegramm MM ist Telegramm an nie verwendete Lok Adresse 80 (GL Adresse 80 wird als eigentliche Adr 0 ausgegeben)
    let mut ddl_idle_tel = self.get_gl_new_tel(80);
    {
      let ddl_daten = ddl_idle_tel.daten.last_mut().unwrap();
      //Adr 80 ist 4 * "O" Trit
      ddl_daten.extend_from_slice(MM_BIT_O);
      ddl_daten.extend_from_slice(MM_BIT_O);
      ddl_daten.extend_from_slice(MM_BIT_O);
      ddl_daten.extend_from_slice(MM_BIT_O);
    }
    //Dann Funktion Off, Speed 0
    self.add_mm1_fnkt_value(&mut ddl_idle_tel, false, 0, false);
    self.complete_mm_paket(&mut ddl_idle_tel);
    Some(ddl_idle_tel)
  }
}

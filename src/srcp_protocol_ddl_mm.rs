use crate::srcp_protocol_ddl::{DdlProtokoll, DdlTel, GLDriveMode};

//SPI Baudrate für Märklin / Motorola Protokoll.
//Diese wäre eigentlich genau 38461 Baud (1 Bit=26us, 1Byte=208us)
const SPI_BAUDRATE_MAERKLIN_LOCO: u32 = 38461;
//Nun macht der RaspberryPI aber leider nach jedem Byte Transfer auf dem SPI Bus 1 Bit Pause :-(
//Er macht diese 1 Bit / Clk Pause nicht mehr im DMA Mode. DMA wird ab Transfers
//von 96 Bytes verwendet (hoffe, das bleibt so ...).
//Um Märklin / Motorola Pakete auf 96 rauf zu bringen, wird folgendes gemacht:
//- die Baudrate Doppelt so gewählt wie notwendig (2 * 38461 für Loks)
//- die Wiederholung ins selbe Paket gepackt.
//- Pause am Anfang und vor Wiederholung mit 0 Bytes gefüllt.
pub const SPI_BAUDRATE_MAERKLIN_LOCO_2: u32 = 2 * SPI_BAUDRATE_MAERKLIN_LOCO;
const SPI_BAUDRATE_MAERKLIN_FUNC_2: u32 = 2 * SPI_BAUDRATE_MAERKLIN_LOCO_2;
//Für Märklin Motorola wird wie folgt kodiert (doppelte Baudrate):
//- Paket mit
//  - 0 -> 0xC0, 0x00, ich habe aber Schaltdekoder, die damit nicht funktionieren sondern einen ein wenig längeren Impuls wollen, also 0xE0, 0x00 ....
//  - 1 -> 0xFF, 0xFC
//  -> 18 * 2 = 36 Bytes
const MM_LEN_PAKET: usize = 18 * 2;
//- 0 Bytes für Pause zwischen Paketen: 3 * (t 2 Bit, 208us / 416us) -> wegen doppelter Baudrate also 2 * 3 * 2 = 12 Bytes 0x00
const MM_LEN_PAUSE_BETWEEN: usize = 2 * 3 * 2;
//- Paket Wiederholung
//- 0 Bytes für Pause nach Paket: 4.2ms (Lok), resp. 2.1ms (Schaltdekoder) -> wegen bei doppleter Baudrate 1 Byte 104us (Lok). 62us (Schalt) = 42 Bytes
const MM_LEN_PAUSE_END: usize = 42;
//Total also 36 + 12 + 36 + 42 = 126 Bytes -> DMA Mode!
const MM_LEN: usize = MM_LEN_PAKET + MM_LEN_PAUSE_BETWEEN + MM_LEN_PAKET + MM_LEN_PAUSE_END;
// Mit doppelter Baudrate je die beiden Bytes für 0 und 1 Übertragung
const MM_BIT_0_0: u8 = 0xC0;
const MM_BIT_0_1: u8 = 0x00;
const MM_BIT_1_0: u8 = 0xFF;
const MM_BIT_1_1: u8 = 0xFC;
static MM_BIT_0: &'static [u8] = &[MM_BIT_0_0, MM_BIT_0_1]; //0
static MM_BIT_1: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1]; //1
static MM_BIT_L: &'static [u8] = &[MM_BIT_0_0, MM_BIT_0_1, MM_BIT_0_0, MM_BIT_0_1]; //00
static MM_BIT_H: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1, MM_BIT_1_0, MM_BIT_1_1]; //11
static MM_BIT_O: &'static [u8] = &[MM_BIT_1_0, MM_BIT_1_1, MM_BIT_0_0, MM_BIT_0_1]; //10
static MM_BIT_U: &'static [u8] = &[MM_BIT_0_0, MM_BIT_0_1, MM_BIT_1_0, MM_BIT_1_1]; //01
                                                                                    // MM2 & 3 Bitmuster für F1-4, letztes Bit 3 ist jeweils der Zustand der Funktion
static MM_F1_4: &'static [u8] = &[0b0011, 0b0100, 0b0110, 0b0111];

//Max. erlaubte Dekoder Adresse (GA und GL)
const MAX_MM_ADRESSE: usize = 80;
//Max. erlaubte GA Adresse (4 GA per Dekoder)
const MAX_MM_GA_ADRESSE: usize = (MAX_MM_ADRESSE + 1) * 4;
/// Implementierung Märklin Motorola Protokoll V1 & 2
#[derive(PartialEq)]
pub enum MmVersion {
  V1, //14 v Stufen, F0, relative Richtung
  V2, //14 v Stufen, F0-4, absolute Richtung
  V3, //28 v Stufen, F0-4, absolute Richtung
}
pub struct MMProtokoll {
  /// Version 1 oder 2, Keine Unterschiede für GA, nur für GL 14 / 28 Fahrstufen, 1 oder 5 Funktionen
  version: MmVersion,
  /// Erkennung Richtungswechsel bei M1, Halten Richtung bei Richtung Nothalt bei M1 und M2
  old_drive_mode: [GLDriveMode; MAX_MM_ADRESSE + 1],
  /// Erkennung Funktionswechsel bei M2 & 3
  old_funktionen: [u64; MAX_MM_ADRESSE + 1],
  /// Speicherung Speed um F1-F4 Pakete für MM2 & 3, die auch den Speed enthalten, kkorrekt erzeugen zu können
  old_speed: [usize; MAX_MM_ADRESSE + 1],
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
    }
  }
  /// MM 4 Adressbits (trinär codiert)
  /// Adresse 80 wird als 0000 gesendet, die eigentliche Adresse 80 ist der Idlestate, Lok 0 gibt es nicht
  /// # Arguments
  /// * ddl_tel - Telegramm, zudem die Adressbits hinzugefügtw erden sollen
  /// * adr_dekoder - Adresse, die ergänzt werden soll, LSB wird zuerst gesendet, 0..80 erlaubt.
  fn add_mm_adr(&self, ddl_tel: &mut DdlTel, mut adr_dekoder: usize) {
    assert!(adr_dekoder < 81);
    if adr_dekoder == 80 {
      adr_dekoder = 0;
    }
    for _ in 0..4 {
      let adr_trit = adr_dekoder % 3;
      adr_dekoder /= 3;
      match adr_trit {
        0 => {
          ddl_tel.daten.extend_from_slice(MM_BIT_L);
        }
        1 => {
          ddl_tel.daten.extend_from_slice(MM_BIT_H);
        }
        2 => {
          ddl_tel.daten.extend_from_slice(MM_BIT_O);
        }
        _ => assert!(false), //Kann nicht vorkommen da Rest der Division mit 3
      }
    }
  }
  /// MM1 Payload 5 Bits, 1 Bit Funktion und 4 Bits Value
  /// # Arguments
  /// * ddl_tel - Telegramm, zudem die Adressbits hinzugefügtw erden sollen
  /// * fnkt - true: Funktionsbit 1, false für 0
  /// * value - 4 Bit Value, LSB wird zuerst gesendet
  fn add_mm1_fnkt_value(&self, ddl_tel: &mut DdlTel, fnkt: bool, mut value: usize) {
    //Zuerst kommt die Funktion
    if fnkt {
      ddl_tel.daten.extend_from_slice(MM_BIT_H);
    } else {
      ddl_tel.daten.extend_from_slice(MM_BIT_L);
    }
    //Dann Value, 4 Bit, LSB als erstes
    assert!(value <= 0x0F);
    for _ in 0..4 {
      if (value & 0x01) == 0 {
        ddl_tel.daten.extend_from_slice(MM_BIT_L);
      } else {
        ddl_tel.daten.extend_from_slice(MM_BIT_H);
      }
      value >>= 1;
    }
  }
  /// MM2 Payload 5 Bits, 1 Bit Funktion und 4 Bits Speed
  /// # Arguments
  /// * ddl_tel - Telegramm, zudem die Adressbits hinzugefügtw erden sollen
  /// * fnkt - true: Funktionsbit 1, false für 0
  /// * speed - 4 Bit Value, LSB wird zuerst gesendet
  /// * dir - Fahrtrichtung, Rückwärts wird ausgewertet, alles andere ist Vorwärts
  fn add_mm2_fnkt_value(
    &self, ddl_tel: &mut DdlTel, fnkt: bool, mut speed: usize, dir: GLDriveMode,
  ) {
    //Zuerst kommt die Funktion
    if fnkt {
      ddl_tel.daten.extend_from_slice(MM_BIT_H);
    } else {
      ddl_tel.daten.extend_from_slice(MM_BIT_L);
    }
    //Bei MM2 wird nur je ein Bit des Paares für die Geschwindigkeit verwendet,
    //das andere für die absolute Richtungsinfo.
    //Parallel zum Speed odieren wir hier die abs. Richtungsbits
    let mut abs_dir: usize;
    if dir == GLDriveMode::Rueckwaerts {
      //Rückwärts
      if speed < 7 {
        abs_dir = 0b1011;
      } else {
        abs_dir = 0b1010;
      }
    } else {
      //Vorwärts
      if speed < 7 {
        abs_dir = 0b0101;
      } else {
        abs_dir = 0b1000;
      }
    }
    //Dann Speed, 4 Bit, LSB als erstes, Verknüpft mit abs. Richtung
    assert!(speed <= 15);
    for _ in 0..4 {
      if (speed & 0x01) == 0 {
        if (abs_dir & 0x01) == 0 {
          ddl_tel.daten.extend_from_slice(MM_BIT_L);
        } else {
          ddl_tel.daten.extend_from_slice(MM_BIT_U);
        }
      } else {
        if (abs_dir & 0x01) == 0 {
          ddl_tel.daten.extend_from_slice(MM_BIT_O);
        } else {
          ddl_tel.daten.extend_from_slice(MM_BIT_H);
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
  fn get_gl_basis_tel_raw(
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, funktionen: u64,
  ) -> DdlTel {
    let mut ddl_tel = DdlTel::new(SPI_BAUDRATE_MAERKLIN_LOCO_2, MM_LEN);
    let mut drive_mode_used = drive_mode;
    let mut speed_used = speed;
    if drive_mode == GLDriveMode::Nothalt {
      //Nothalt gibt es nicht -> kein Richtungswechsel, Speed = 0
      drive_mode_used = self.old_drive_mode[adr];
      speed_used = 0;
    }
    if speed_used == 1 {
      speed_used = 2; //Speed 1 ist Richtungswechsel
    }
    self.add_mm_adr(&mut ddl_tel, adr);
    match self.version {
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
          &mut ddl_tel,
          (funktionen & 0x01) != 0, //F0
          speed_used,
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
          &mut ddl_tel,
          (funktionen & 0x01) != 0, //F0
          speed_used,
          drive_mode_used,
        );
      }
      MmVersion::V3 => {
        //28 Speeds, F0-4, abs. Richtung, analog V2, zusätzlicher Speed Schritt über 2. Bit F0
        //Max Speed Kontrolle
        if speed_used > 28 {
          speed_used = 28;
        }
        let speed_halfstep = (speed_used % 2) == 0;
        //Nun Speed auf den Range 0..15 wie für V1,2 zurück skalieren
        let speed = ((speed_used + 1) / 2) + 1;
        //Fahren mit abs. Richtung
        self.add_mm2_fnkt_value(
          &mut ddl_tel,
          (funktionen & 0x01) != 0, //F0
          speed,
          drive_mode_used,
        );
        //2. Bit in F0 wird für zwischen Speedschritt verwendet
        if speed_halfstep {
          let f0_bitfolge = if (funktionen & 0x01) != 0 {
            MM_BIT_O
          } else {
            MM_BIT_U
          };
          let faktor_baudrate = MM_BIT_0.len();
          for i in 0..f0_bitfolge.len() {
            ddl_tel.daten[faktor_baudrate * 8 + i] = f0_bitfolge[i];
          }
        }
      }
    }
    self.old_drive_mode[adr] = drive_mode_used;
    self.old_speed[adr] = speed_used;
    //und noch F0 übernehmen
    self.old_funktionen[adr] &= !1; //F0 löschen
    self.old_funktionen[adr] |= funktionen & 1; //und neu übernehmen löschen
    ddl_tel
  }

  /// MM Paket vervollständigen:
  /// - Pause zwischen den beiden Paketen
  /// - Paketwiederholung
  /// - Pause am Schluss
  fn complete_mm_paket(&self, ddl_tel: &mut DdlTel) {
    //Pause zwischen den beiden Paketen ergänzen
    ddl_tel
      .daten
      .resize(ddl_tel.daten.len() + MM_LEN_PAUSE_BETWEEN, 0);
    //Wiederholung
    let tel: Vec<u8> = ddl_tel.daten[0..MM_LEN_PAKET].to_vec();
    ddl_tel.daten.extend(&tel);
    //Pause am Schluss
    ddl_tel
      .daten
      .resize(ddl_tel.daten.len() + MM_LEN_PAUSE_END, 0);
  }
}
impl DdlProtokoll for MMProtokoll {
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
    }
  }
  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize {
    //Es ist nur F0 im Tel. mit Speed und Richtung vorhanden
    1
  }
  /// Erzeugt das Basis Telegramm für GL.
  /// - Fahren
  /// - Basisfunktionen F0 bis "get_Anz_F_Basis"
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * drive_mode - Fahrtrichtung / Nothalt
  /// * speed - aktuelle Geschwindigkeit
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis". Es wedren hier nur diese Funktionen übernommen!
  fn get_gl_basis_tel(
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, funktionen: u64,
  ) -> DdlTel {
    let mut ddl_tel = self.get_gl_basis_tel_raw(adr, drive_mode, speed, funktionen);
    self.complete_mm_paket(&mut ddl_tel);
    //Und versenden
    ddl_tel
  }
  /// Erzeugt das / die Fx Zusatztelegramm(e) für GL.
  /// - Funktionen nach "get_Anz_F_Basis"
  /// Liefert None, wenn kein Zusatztelegramm vorhanden ist. Das ist bei MM1 immer der Fall.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * refresh - Wenn false werden nur Telegramme für Funktionen, die geändert haben, erzeugt
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis"
  fn get_gl_zusatz_tel(&mut self, adr: usize, refresh: bool, funktionen: u64) -> Option<DdlTel> {
    if self.version == MmVersion::V1 {
      return None;
    }
    //Das ganze Paket an Telegrammen für F1-4
    let mut ddl_tel_f1_4 = DdlTel::new(SPI_BAUDRATE_MAERKLIN_LOCO_2, MM_LEN * 4);
    //Nun noch F1-4, jedoch nur bei Veränderung sofort senden
    for i in 1..self.get_gl_anz_f() {
      //Als Basis Standard Fahren Telegramm verwenden und dieses dann auf F1-4 ändern
      let mut ddl_tel = self.get_gl_basis_tel_raw(
        adr,
        self.old_drive_mode[adr],
        self.old_speed[adr],
        funktionen,
      );
      let mask: u64 = 1 << i;
      if (((self.old_funktionen[adr] ^ funktionen) & mask) != 0) || refresh {
        //Veränderung oder immer verlangt
        let mut fx_bits = MM_F1_4[i - 1];
        //Zustand der Funktion ergänzen
        if (funktionen & mask) != 0 {
          fx_bits |= 0b1000;
        }
        for bit in 0..4 {
          //Bit 11 13 15. Da wegen doppelter Baurate 2 Byte pro Bit nochmals * 2
          let faktor_baudrate = MM_BIT_0.len();
          for j in 0..faktor_baudrate {
            ddl_tel.daten[faktor_baudrate * (11 + bit * 2) + j] = if (fx_bits & 0b0001) == 0 {
              MM_BIT_0[j]
            } else {
              MM_BIT_1[j]
            };
          }
          fx_bits >>= 1;
        }
        self.complete_mm_paket(&mut ddl_tel);
        ddl_tel_f1_4.daten.append(&mut ddl_tel.daten);
      }
    }
    self.old_funktionen[adr] = funktionen;
    if ddl_tel_f1_4.daten.len() > 0 {
      Some(ddl_tel_f1_4)
    } else {
      None
    }
  }
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder, 0 oder 1
  /// * value - Gewünschter Zustand des Port Ein/Aus
  fn get_ga_tel(&self, adr: usize, port: usize, value: bool) -> DdlTel {
    //Dekoderadresse: 4 Ausgangspaare auf Dekoder, deshalb adr/4
    let adr_dekoder = (adr - 1) >> 2;
    //Subadresse auf Dekoder ist welches der 4 Paare plus Port
    let sub_adr = (((adr - 1) & 3) << 1) + (port & 1);
    let mut ddl_tel = DdlTel::new(SPI_BAUDRATE_MAERKLIN_FUNC_2, MM_LEN);
    self.add_mm_adr(&mut ddl_tel, adr_dekoder);
    self.add_mm1_fnkt_value(
      &mut ddl_tel,
      false,
      sub_adr + (if value { 0x08 } else { 0x00 }), //Value ist das 4. Bit
    );
    self.complete_mm_paket(&mut ddl_tel);
    //Und versenden
    ddl_tel
  }

  /// Liefert das Idle Telegramm dieses Protokolles
  fn get_idle_tel(&self) -> DdlTel {
    //Idle Telegramm MM ist Telegramm an nie verwendete Lok Adresse 80 (GL Adresse 80 wird als eigentliche Adr 0 ausgegeben)
    let mut ddl_idle_tel = DdlTel::new(SPI_BAUDRATE_MAERKLIN_LOCO_2, MM_LEN);
    //Adr 80 ist 4 * "O" Trit
    ddl_idle_tel.daten.extend_from_slice(MM_BIT_O);
    ddl_idle_tel.daten.extend_from_slice(MM_BIT_O);
    ddl_idle_tel.daten.extend_from_slice(MM_BIT_O);
    ddl_idle_tel.daten.extend_from_slice(MM_BIT_O);
    //Dann Funktion L, Speed 0
    self.add_mm1_fnkt_value(&mut ddl_idle_tel, false, 0);
    self.complete_mm_paket(&mut ddl_idle_tel);
    ddl_idle_tel
  }
}

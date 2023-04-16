use crate::srcp_protocol_ddl::{DdlProtokoll, DdlTel, GLDriveMode};

//SPI Baudrate für DCC/NMRA.
//Diese wird so gewählt, dass ein kurzer 58/58us Impuls (logisch 1) in einem Byte (0xF0) ausgegeben werden kann,
//ein langer 116/116us Impuls wird dann als 2 Bytes (0xFF, 0x00) ausgegeben.
//Damit hätten wird für 8 Bit also 116us -> 1 Bit 14.5us -> 68966 Baud.
//Damit werden NMRA/DCC Pakete inkl. der führenden Sync. Bytes leider nicht immer >= 96 Bytes -> DMA Mode und keine Pause nach 8 Bits
const SPI_BAUDRATE_NMRA: u32 = 68966;
//Deshalb wird auch hier die doppelte Baudrate verwendet und dann wie folgt kodiert:
//1: 0xFF, 0x00
//0: 0xFF, 0xFF, 0x00, 0x00
const SPI_BAUDRATE_NMRA_2: u32 = SPI_BAUDRATE_NMRA * 2;

static DCC_BIT_1: &'static [u8] = &[0xFF, 0x00]; //1
static DCC_BIT_0: &'static [u8] = &[0xFF, 0xFF, 0x00, 0x00]; //0

///Max. erlaubte GL Kurz-Adresse (V1, 7 Bit)
const MAX_DCC_GL_ADRESSE_KURZ: usize = 127;
///Max. erlaubte GL Lang-Adresse (V2, 14 Bit)
const MAX_DCC_GL_ADRESSE_LANG: usize = 10239;
///Max. erlaubte GA Adresse gem. NMRA S-9.2.1 Dekoder mit 4*2 Ausgängen, 9 Bit für Dekoder Adresse
///Dekoderadresse 0x1FF ist reserviert für Broadcast, es bleiben also inkl. Adr 0 511 Dekoderadressen.
const MAX_DCC_GA_ADRESSE: usize = 511 * 4;
///Anzahl sync. Bits
const ANZ_DCC_SYNC: usize = 16;

///Max. Tel. Längen, Annahme: alle variablen Bits sind 0
///1 Byte: 1111111111111111 0 00000000 0 00000000 1 -> 17*1, 18*0
///2 Byte: 1111111111111111 0 00000000 0 00000000 0 00000000 1 -> 17*1, 27*0
///3 Byte: 1111111111111111 0 00000000 0 00000000 0 00000000 0 00000000 1 -> 17*1, 36*0
const DCC_MAX_LEN_BASIS: usize = 17 * 2;
const DCC_MAX_LEN_PRO_BYTE: usize = 9 * 4;
//Berechnung max. Länge ist dann: DCC_MAX_LEN_BASIS + <Anz Bytes> * DCC_MAX_LEN_PRO_BYTE
///Minimale Zeit zwischen zwei Telegrammen an den selben Dekoder ohne Sync Bits ist 5ms.
///Wird erreicht, durch mehr Syncbits wenn 2 Pakete (Fahren und F0-F5) unmittelbar nacheinandner kommen.
const PAUSE_ZWISCHEN: usize =
  (5000 / (8000000 / (SPI_BAUDRATE_NMRA as usize))) + 1 - (ANZ_DCC_SYNC as usize);

//DCC Instruktionen erstes Datenbyte. Die 3 MSB Bits sind relevant
///DCC Advanced Operations
const DCC_INST_ADVOP: u8 = 0b00100000;
///DCC Advanced Operations -> 128 Speed controll (die unteren 5 Bit zu DCC_INST_ADVOP)
const DCC_INST_ADVOP_128_SPEED: u8 = 0b00011111;
///DCC Instr. Drive Reverse
const DCC_INST_DRIVE_REVERSE: u8 = 0b01000000;
///DCC Instr. Drive Forward
const DCC_INST_DRIVE_FORWARD: u8 = 0b01100000;
///DCC Instr. F0-F4
const DCC_INST_F0_F4: u8 = 0b10000000;
///DCC Instr. F5-F12
const DCC_INST_F5_F12: u8 = 0b10100000;
///DCC Instr. Expansion
const DCC_INST_EXP: u8 = 0b11000000;
///DCC Instr. Expansion -> F13-F20 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F13_F20: u8 = 0b00011110;
///DCC Instr. Expansion -> F21-F28 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F21_F28: u8 = 0b00011111;
///DCC Instr. Expansion -> F29-F36 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F29_F36: u8 = 0b00011000;
///DCC Instr. Expansion -> F37-F44 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F37_F44: u8 = 0b00011001;
///DCC Instr. Expansion -> F45-F52 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F45_F52: u8 = 0b00011010;
///DCC Instr. Expansion -> F53-F60 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F53_F60: u8 = 0b00011011;
///DCC Instr. Expansion -> F61-F68 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F61_F68: u8 = 0b00011100;

//Grenzen für Speed Steps bis und mit diesem Wert
const SPEED_STEP_4BIT: usize = 14;
const SPEED_STEP_5BIT: usize = 29;

pub enum DccVersion {
  V1, //Kurze Lokadresse bis 127
  V2, //Lange Lokadresse 128 bis 10239. Bis 127 wird gemäss DCC Standard automatisch auch hier immer mit der kurzen Adresse gearbeitet
}

pub struct DccProtokoll {
  /// Version 1 oder 2, Keine Unterschiede für GA, nur kurze Lange GL Adr.
  version: DccVersion,
  /// Halten Richtung bei Richtung Nothalt
  old_drive_mode: [GLDriveMode; MAX_DCC_GL_ADRESSE_LANG + 1],
  /// Erkennung Funktionswechsel für die nicht immer gesendeten höheren Fx
  old_funktionen: [u64; MAX_DCC_GL_ADRESSE_LANG + 1],
}

impl DccProtokoll {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * version - V1 oder V2
  pub fn from(version: DccVersion) -> DccProtokoll {
    DccProtokoll {
      version,
      old_drive_mode: [GLDriveMode::Vorwaerts; MAX_DCC_GL_ADRESSE_LANG + 1],
      old_funktionen: [0; MAX_DCC_GL_ADRESSE_LANG + 1],
    }
  }

  /// Fügt Start Syncmuster 16*1 mit abschliessendem 0 zum Telegramm
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Sync. ergänzt werden soll
  fn add_sync(&self, ddl_tel: &mut DdlTel) {
    for _ in 0..ANZ_DCC_SYNC {
      ddl_tel.daten.extend_from_slice(DCC_BIT_1);
    }
    ddl_tel.daten.extend_from_slice(DCC_BIT_0);
  }

  /// Fügt ein Byte zum DDL DCC Telegramm hinzu und aktualisiert die Prüfsumme (exor)
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem ein Byte (MSB zuerst) hinzugefügt werden soll
  /// * value - Byte, das hinzugefügt werden soll
  /// * xor - Alter und nachher aktueller xor Wert
  fn add_byte(&self, ddl_tel: &mut DdlTel, value: u8, xor: &mut u8) {
    for i in (0..8).rev() {
      //Geht von 7 bis 0
      ddl_tel.daten.extend_from_slice(if (value & (1 << i)) == 0 {
        DCC_BIT_0
      } else {
        DCC_BIT_1
      });
      *xor ^= value;
    }
  }
  /// Fügt die Addresse (1 Byte bis 127, 2 Byte wenn grösser) mit abschliessendem 0 hinzu
  /// Liefert die aktuelle Prüfsumme (exor) nach Adresse zurück
  fn add_adr(&self, ddl_tel: &mut DdlTel, adr: usize) -> u8 {
    let mut xor: u8 = 0;
    if adr <= MAX_DCC_GL_ADRESSE_KURZ {
      self.add_byte(ddl_tel, (adr & 0xFF).try_into().unwrap(), &mut xor);
    } else {
      //14 Bit Adr. Es kommen 6 bit MSB ins erste byte, dann 8 Bit bis zum LSB
      //Start des ersten Adr. Bytes mit 11
      let adr_msb: u8 = (0b11000000 | ((adr >> 8) & 0xFF)).try_into().unwrap();
      //Dann die 6 Bits ab MSB, mit MAX_DCC_GL_ADRESSE_LANG ist sichergestellt, dass die ersten 6 Bits
      //nur von 000000 bis 100111 gehen können
      self.add_byte(ddl_tel, adr_msb, &mut xor);
      //Nach einem Byte abschliessendes 0
      ddl_tel.daten.extend_from_slice(DCC_BIT_0);
      //Und noch ein byte mit den 8 LSB Bits
      self.add_byte(ddl_tel, (adr & 0xFF).try_into().unwrap(), &mut xor);
    }
    //Nach einem Byte abschliessendes 0
    ddl_tel.daten.extend_from_slice(DCC_BIT_0);
    xor
  }
}
impl DdlProtokoll for DccProtokoll {
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> usize {
    match self.version {
      DccVersion::V1 => MAX_DCC_GL_ADRESSE_KURZ,
      DccVersion::V2 => MAX_DCC_GL_ADRESSE_LANG,
    }
  }
  /// Liefert die max. erlaubte Schaltmoduladdresse
  fn get_ga_max_adr(&self) -> usize {
    MAX_DCC_GA_ADRESSE
  }
  /// Liefert die max. Anzahl der unterstützten Funktionen
  fn get_gl_anz_f(&self) -> usize {
    64 //Eigentlich kann DCC Total 69 (F0-F68), im Moment reicht mir das aber, die Funktionen werden in ganz srcp_rust in einem u64 verwaltet
  }
  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize {
    //F0 bis F5 sind 1 Byte Kommandos, Zustand wird direkt als Bits gesendet. Diese nehmen wir zur Basis die immer gesendet werden.
    5
  }
  /// Erzeugt das Basis Telegramm für GL.
  /// - Fahren
  /// - Basisfunktionen F0 bis "get_Anz_F_Basis"
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * drive_mode - Fahrtrichtung / Nothalt
  /// * speed - aktuelle Geschwindigkeit
  /// * speed_steps - Anzahl Speed Steps die verwendet werden soll. Für DCC relevant: bis 14, bis 29, wenn grösser dann 128
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis". Es wedren hier nur diese Funktionen übernommen!
  fn get_gl_basis_tel(
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, speed_steps: usize,
    funktionen: u64,
  ) -> DdlTel {
    //Prüfung Gültigkeit Adresse
    assert!(adr <= self.get_gl_max_adr(), "DCC GL Adresse zu gross");
    //Gültigkeit speed prüfen
    assert!(speed <= speed_steps, "DCC Speed > Speed Steps");
    //Drivemode für Richtung, wenn Nothalt dann der letzte
    let drive_mode_used: GLDriveMode = if drive_mode == GLDriveMode::Nothalt {
      self.old_drive_mode[adr]
    } else {
      drive_mode
    };
    self.old_drive_mode[adr] = drive_mode_used;
    //Speed 1 = Nothalt
    let speed_used = if drive_mode == GLDriveMode::Nothalt {
      1
    } else {
      if speed == 1 {
        2
      } else {
        speed
      }
    };
    //Telegramm mit Sync
    let mut anz_bytes: usize = 2 + 2; //Minimales Teleramm für GL + F0-F5
    if adr > MAX_DCC_GL_ADRESSE_KURZ {
      anz_bytes += 2; //2. Adressbyte für Tel. Fahren und F0-F5
    }
    if speed_steps > SPEED_STEP_5BIT {
      anz_bytes += 1; //Extrabyte für Speed
    }
    let mut ddl_tel = DdlTel::new(
      SPI_BAUDRATE_NMRA_2,
      2 * DCC_MAX_LEN_BASIS + anz_bytes * DCC_MAX_LEN_PRO_BYTE,
    );
    self.add_sync(&mut ddl_tel);
    //Addresse in 1 oder 2 Bytes
    let mut xor = self.add_adr(&mut ddl_tel, adr);
    //Kommando ist nun abhängig von den gewünschten Anzahl Speed Steps
    if speed_steps > SPEED_STEP_5BIT {
      //Kommando Speed mit 128 Steps und extra Byte mit Speed
      self.add_byte(
        &mut ddl_tel,
        DCC_INST_ADVOP | DCC_INST_ADVOP_128_SPEED,
        &mut xor,
      );
      //Nach einem Byte abschliessendes 0
      ddl_tel.daten.extend_from_slice(DCC_BIT_0);
      //MSB Richtung und 7 Bit Speed
      let speed_byte: u8 = ((speed & 0b01111111)
        | if drive_mode_used == GLDriveMode::Vorwaerts {
          0b10000000
        } else {
          0b00000000
        })
      .try_into()
      .unwrap();
      self.add_byte(&mut ddl_tel, speed_byte, &mut xor);
    } else {
      let mut speed_byte: u8 = if drive_mode_used == GLDriveMode::Vorwaerts {
        DCC_INST_DRIVE_FORWARD
      } else {
        DCC_INST_DRIVE_REVERSE
      };
      if speed_steps > SPEED_STEP_4BIT {
        //Kommando Fahren vorwärts oder rückwärts plus 5 Bit Speed
        //Bit0-3 -> Bit 1-4 Speed, Bit4 -> Bit 0 Speed
        speed_byte |= TryInto::<u8>::try_into((speed_used >> 1) & 0b00001111).unwrap()
          | TryInto::<u8>::try_into((speed_used << 4) & 0b00010000).unwrap();
      } else {
        //Kommando Fahren vorwärts oder rückwärts plus 4 Bit Speed
        //Bit0-3 -> Bit 0-3 Speed
        //Bit 4 ist F0
        speed_byte |= TryInto::<u8>::try_into(speed_used & 0b00001111).unwrap()
          | TryInto::<u8>::try_into(((funktionen & 1) << 4) & 0b00010000).unwrap();
      }
      self.add_byte(&mut ddl_tel, speed_byte, &mut xor);
    }
    //Nach einem Byte abschliessendes 0
    ddl_tel.daten.extend_from_slice(DCC_BIT_0);
    //Checksumme ergänzen
    self.add_byte(&mut ddl_tel, xor, &mut xor);
    //Tel mit abschliessenden mit 1 Bit
    ddl_tel.daten.extend_from_slice(DCC_BIT_1);
    //Pause PAUSE_ZWISCHEN

    //Und nun noch F0 bis F5
    //Addresse in 1 oder 2 Bytes
    xor = self.add_adr(&mut ddl_tel, adr);

    //Und versenden
    ddl_tel
  }
  /// Erzeugt das / die Fx Zusatztelegramm(e) für GL.
  /// - Funktionen nach "get_Anz_F_Basis"
  /// Liefert None, wenn kein Zusatztelegramm vorhanden ist.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * refresh - Wenn false werden nur Telegramme für Funktionen, die geändert haben, erzeugt
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis"
  fn get_gl_zusatz_tel(&mut self, adr: usize, refresh: bool, funktionen: u64) -> Option<DdlTel> {
    self.old_funktionen[adr] = funktionen;
    None
  }
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder, 0 oder 1
  /// * value - Gewünschter Zustand des Port Ein/Aus
  fn get_ga_tel(&self, adr: usize, port: usize, value: bool) -> DdlTel {
    //Dekoderadresse: 4 Ausgangspaare auf Dekoder, deshalb adr/4
    let mut ddl_tel = DdlTel::new(
      SPI_BAUDRATE_NMRA_2,
      DCC_MAX_LEN_BASIS + 2 * DCC_MAX_LEN_PRO_BYTE,
    );
    //Und versenden
    ddl_tel
  }

  /// Liefert das Idle Telegramm dieses Protokolles
  fn get_idle_tel(&self) -> DdlTel {
    //DCC Idle Telegramm: 1111111111111111 0 11111111 0 00000000 0 11111111 1
    let mut ddl_idle_tel = DdlTel::new(
      SPI_BAUDRATE_NMRA_2,
      DCC_MAX_LEN_BASIS + 2 * DCC_MAX_LEN_PRO_BYTE,
    );
    ddl_idle_tel
  }
}

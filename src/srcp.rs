//! srcp Protokoll Implementierung
//! - Info mode
//! - Command mode
//!   - INIT
//!   - SET
//!
//! INI File:
//! [srcp]
//! port = xxxxxx

use std::{
  collections::HashMap,
  io::{Read, Write},
  net::{TcpListener, TcpStream},
  sync::{
    mpsc::{self, Receiver, Sender},
    Mutex,
  },
  thread,
  time::{Duration, SystemTime, UNIX_EPOCH},
};

use log::{error, info, warn};
use splitty::split_unquoted_char;

use crate::srcp_server_types::{Message, SRCPMessage};

//Unterstützte SRCP version
const SRCP_VERSION: &'static str = "0.8.4";

//Verwaltung Sender und Session
struct SenderSession {
  sender: Sender<SRCPMessage>,
  session_id: u32,
}
//Info Messages können für Info und Command clients relevant sein
struct InfoSenderForClient {
  info_client: Vec<SenderSession>,
  command_client: Vec<SenderSession>,
}
//Verwaltung aller Sender zu allen angemeldeten SRCP Clients
static ALLE_SRCP_INFO_SENDER: Mutex<InfoSenderForClient> = Mutex::new(InfoSenderForClient {
  command_client: Vec::new(),
  info_client: Vec::new(),
});

//enum für SRCP Command- oder Infomode
#[derive(Debug)]
enum SrcpMode {
  Command,
  Info,
}

/// Read line function die tolerant gegenüber nicht ASCII Zeichen ist, diese werden ignoriert.
/// Es wird jeweils bis \n gelesen. Blockiert solange kein \n gelesen wurde oder Verbindung abbricht.
/// Liefert Err bei Verbindungsabbruch
/// Es wird IMMER alles in Grossbuchstaben zurück geliefert.
/// # Arguments
/// * client_stream - TCP Stream von dem gelesen werden soll
/// * line - Gelesene Zeile
fn read_line(mut client_stream: &TcpStream, line: &mut String) -> Result<(), ()> {
  let mut buffer: [u8; 1] = [0; 1];
  line.clear();
  loop {
    client_stream.read_exact(&mut buffer).or(Err(()))?;
    match buffer[0] {
      b'\n' => break,
      b' '..=b'~' => line.push(
        char::from_u32(buffer[0].into())
          .unwrap()
          .to_ascii_uppercase(),
      ),
      _ => {} //Ignorieren
    }
  }
  Ok(())
}

/// SRCP Message zum Client senden
/// Liefert Err bei Verbindungsabbruch
/// # Arguments
/// * client_stream - TCP Stream von dem gelesen werden soll
/// * msg - Die zu sendene Message. Diese wird am Anfang mit Timestamp ergänzt und am Schluss mit\n
fn send_srcp_message(mut client_stream: &TcpStream, msg: &str) -> Result<(), String> {
  let time = SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .expect("Time went backwards");
  let text = time.as_secs().to_string()
    + "."
    + format!("{:0>3}", time.subsec_millis()).as_str()
    + " "
    + msg
    + "\n";
  client_stream
    .write(text.as_bytes())
    .or(Err("SRCP Write to client Error"))?;
  Ok(())
}

/// SRCP Error Message zum Client senden
/// Liefert Err bei Verbindungsabbruch
/// # Arguments
/// * client_stream - TCP Stream von dem gelesen werden soll
/// * err_code - SRCP Errorcode
/// * msg - Error Message
fn send_srcp_error(client_stream: &TcpStream, err_code: &str, msg: &str) -> Result<(), String> {
  send_srcp_message(client_stream, &format!("{} ERROR {}", err_code, msg))
}

/// SRCP Server Handshake mit Client.
/// Liefert den gewünschten SRCP Mode oder Error
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
fn handle_srcp_handshake(
  mut client_stream: &TcpStream, session_id: u32,
) -> Result<SrcpMode, String> {
  let mut line = String::new();
  //SRCP Willkommensmessage senden
  //srcpd Vx.x.x; SRCP x.x.x
  client_stream
    .write(
      format!(
        "srcpd V{}; SRCP {}\n",
        env!("CARGO_PKG_VERSION"),
        SRCP_VERSION
      )
      .as_bytes(),
    )
    .or(Err("SRCP Client Write fail"))?;
  loop {
    //Warten auf gewünschten Mode
    if read_line(client_stream, &mut line).is_err() {
      return Err(format!("SRCP read_line Error"));
    }
    let mode = match line.to_uppercase().as_str() {
      "SET CONNECTIONMODE SRCP COMMAND" => SrcpMode::Command,
      "SET CONNECTIONMODE SRCP INFO" => SrcpMode::Info,
      _ => {
        if line.starts_with("SET PROTOCOL SRCP") {
          //Wird ignoriert, eibfach mit OK beantworten
          send_srcp_message(client_stream, "201 OK PROTOCOL SRCP")?;
        } else {
          warn!("Ungültiges SRCP Kommando empfangen: {}", line);
          send_srcp_error(client_stream, "401", "unsupported connection mode")?;
        }
        continue;
      }
    };
    send_srcp_message(client_stream, "202 OK CONNECTIONMODE")?;
    //Warten auf GO
    read_line(client_stream, &mut line).or(Err("SRCP read_line Errro"))?;
    match line.to_uppercase().as_str() {
      "GO" => (),
      _ => {
        return Err(format!("SRCP GO erwartet: {}", line));
      }
    };
    //Start neue Session
    send_srcp_message(client_stream, format!("200 OK GO {}", session_id).as_str())?;
    return Ok(mode);
  }
}

/// Info Mode SRCP Client bedienen
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn handle_srcp_infomode(
  mut client_stream: &TcpStream, session_id: u32, all_cmd_tx: &HashMap<usize, Sender<Message>>,
) {
  //No blocking read um Rx Buffer leeren zu können
  client_stream
    .set_nonblocking(true)
    .expect("handle_srcp_infomode set_nonblocking call failed");
  //Channel zum Empfang von Info Message aufbauen und anmelden
  let (info_tx, info_rx) = mpsc::channel();
  //und anmelden
  {
    let mut guard = ALLE_SRCP_INFO_SENDER.lock().unwrap();
    let prot_alle_info_sender = &mut *guard; // take a &mut borrow of the value
    prot_alle_info_sender.info_client.push(SenderSession {
      sender: info_tx,
      session_id: session_id,
    });
  }
  //Allen Servern den neuen Info Mode Client mitteilen so dass diese ein Update aller Zustände senden können
  let message = Message::new_info_client(session_id);
  for (_, sender) in all_cmd_tx {
    sender
      .send(message.clone())
      .expect("handle_srcp_infomode Error Send to Server fail");
  }
  //Und ab jetzt einfach alle Info Meldungen weitersenden
  loop {
    let srcp_msg = info_rx
      .recv()
      .expect("handle_srcp_infomode Error recv")
      .to_string();
    if send_srcp_message(client_stream, srcp_msg.as_str()).is_err() {
      //Abbruch, Client ist gestorben
      break;
    }
    //Für Info Verbindungen wird nichts empfangen. Zur Sicherheit Eingangsbuffer löschen
    let mut buf = vec![];
    let _ = client_stream.read_to_end(&mut buf); //Alle Fehler ignorieren
  }
  info!("SRCP Info Client {} beendet", session_id);
}

/// Command Mode SRCP Client bedienen
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn handle_srcp_commandmode(
  client_stream: &TcpStream, session_id: u32, all_cmd_tx: &HashMap<usize, Sender<Message>>,
) {
  //Channel zum Empfang von Info Message aufbauen und anmelden
  let (info_tx, info_rx) = mpsc::channel();
  //und anmelden
  {
    let mut guard = ALLE_SRCP_INFO_SENDER.lock().unwrap();
    let prot_alle_info_sender = &mut *guard; // take a &mut borrow of the value
    prot_alle_info_sender.command_client.push(SenderSession {
      sender: info_tx,
      session_id: session_id,
    });
  }
  //Solange auf Kommandos warten, auswerten und weitersenden, auf Antwort warten und zurück senden bis der Client gestorben ist
  let mut line = String::new();
  loop {
    //Kommando lesen
    if read_line(client_stream, &mut line).is_err() {
      break;
    }
    //Jedes Kommando muss folgendes Format haben:
    //<cmd> <busnr> <dev_group> [<param1> [<param2> ....]]
    let cmd_parts: Vec<&str> = split_unquoted_char(line.as_str(), ' ')
      .unwrap_quotes(true)
      .collect();
    //Empfangsqueue sollte leer sein.
    //Wenn nicht, dann gab es mal mehr als eine Antwort auf eine Kommando, was nicht sein sollte...
    while let Ok(msg) = info_rx.try_recv() {
      warn!(
        "handle_srcp_commandmode: Nicht erwartete Message in info_rx: {}",
        msg.to_string()
      );
    }
    //Kommando Auswerten
    match SRCPMessage::from(session_id, &cmd_parts) {
      Ok(srcp_msg) => {
        //Prüfen ob verlangter Bus existiert
        match all_cmd_tx.get(&srcp_msg.bus) {
          Some(sender) => {
            sender.send(Message::new_srcpmessage(srcp_msg)).unwrap();
            //Warten auf Antwort
            if let Ok(msg) = info_rx.recv_timeout(Duration::from_millis(500)) {
              if let Err(msg) = send_srcp_message(client_stream, msg.to_string().as_str()) {
                warn!("{}", msg);
                break;
              }
            } else {
              warn!(
                "Keine Antwort von SRCP Server an Bus {} erhalten.",
                cmd_parts[1]
              );
              if let Err(msg) = send_srcp_error(client_stream, "417", "timeout") {
                warn!("{}", msg);
                break;
              }
            }
          }
          None => {
            if let Err(msg) = send_srcp_error(client_stream, "412", "wrong value") {
              warn!("{}", msg);
              break;
            }
          }
        }
      }
      Err((errcode, errmsg)) => {
        info!("Ungültiger Befehl empfangen: {}", line);
        if let Err(msg) = send_srcp_error(client_stream, errcode, errmsg) {
          warn!("{}", msg);
          break;
        }
      }
    }
  }
  info!("SRCP Command Client {} beendet", session_id);
}

/// SRCP Server Thread für einen Client
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn handle_srcp_connection(
  client_stream: &TcpStream, session_id: u32, all_cmd_tx: HashMap<usize, Sender<Message>>,
) {
  match handle_srcp_handshake(client_stream, session_id) {
    Err(msg) => {
      error!("SRCP Handshake Error: {}", msg);
      return;
    }
    Ok(mode) => {
      info!(
        "Neuer Client SRCP Mode={:?} session_id={}",
        mode, session_id
      );
      match mode {
        SrcpMode::Command => handle_srcp_commandmode(client_stream, session_id, &all_cmd_tx),
        SrcpMode::Info => handle_srcp_infomode(client_stream, session_id, &all_cmd_tx),
      }
    }
  }
}

/// SRCP Server der auf eingehende Verbindungen wartet, diese entgegennimmt und für jede Verbindung
/// einen Rx und Tx Thread startet
/// # Arguments
/// * port - TCP Port auf dem der Server gestartet werden soll
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn srcp_server(port: u16, all_cmd_tx: &HashMap<usize, Sender<Message>>) -> ! {
  let server_adr = format!("0.0.0.0:{}", port);
  info!("Start SRCP Server: {}", server_adr);
  let listener = TcpListener::bind(server_adr).expect(
    format!(
      "SRCP Server konnte nicht auf Port {} gestartet werden",
      port
    )
    .as_str(),
  );
  let mut session_id: u32 = 0;
  loop {
    info!("Warte auf SRCP Server Client");
    let (client_stream, addr) = listener.accept().expect("SRCP Server Accept fail");
    info!("SRCP Server neuer Client:{}", addr);
    session_id = session_id + 1;
    //Alle Sender müssen geklont werden damit sie im anderen Thread verwendet werden können
    let all_cmd_tx_kopie = all_cmd_tx.clone();
    //Neuer Thread für diesen Client starten
    thread::Builder::new()
      .name(format!(
        "SRCP_Client_Thread Session={} Client={}",
        session_id, addr
      ))
      .spawn(move || handle_srcp_connection(&client_stream, session_id, all_cmd_tx_kopie))
      .unwrap();
  }
}

/// Senden einer SRCP Info Message an eine Clientgruppe
/// Wenn eine Message nicht versendet werden konnte, dann wird der entsprechende Client gelöscht.
/// Wenn in der Message eine Session ID vorhanden ist, dan wird die Message nur an diesen Client gesendet.
/// # Arguments
/// * clients - Die Clientsgruppe
/// * msg - Die zu versendende Message
/// * nur_mit_session - Nur versenden wenn Sessin ID vorhanden
fn send_info_msg_for_client_group(
  clients: &mut Vec<SenderSession>, srcp_message: &SRCPMessage, nur_mit_session: bool,
) {
  if srcp_message.session_id.is_none() && nur_mit_session {
    return;
  }
  let mut i = 0;
  while i < clients.len() {
    if srcp_message.session_id.is_none()
      || (clients[i].session_id == srcp_message.session_id.unwrap())
    {
      if clients[i].sender.send(srcp_message.clone()).is_err() {
        //Diesen Client gibt es nicht mehr
        info!(
          "dispachter_srcp_info delete Client session_id={}",
          clients[i].session_id
        );
        clients.remove(i);
      } else {
        i += 1;
      }
    } else {
      i += 1;
    }
  }
}

/// Dispatcher für alle SRCP Info Messages von allen Servern zu Weiterleitung an alle
/// aktuell angemeldeten Info Clients
/// # Arguments
/// * info_rx - Channel über die die Info Messages empfangen werden
fn dispachter_srcp_info(info_rx: Receiver<SRCPMessage>) {
  loop {
    let msg = info_rx
      .recv()
      .expect("Error: dispachter_srcp_info info_rx.recv() fail");
    {
      //Info/Ok/Err Message an alle oder einen angemeldeten SRCP Info Clients versenden
      let mut guard = ALLE_SRCP_INFO_SENDER.lock().unwrap();
      let prot_alle_info_sender = &mut *guard; // take a &mut borrow of the value

      //Zuerst alle Info Clients abarbeiten
      send_info_msg_for_client_group(&mut prot_alle_info_sender.info_client, &msg, false);
      //Dann alle Command Clients, hier aber nur wenn Session ID angegeben ist
      send_info_msg_for_client_group(&mut prot_alle_info_sender.command_client, &msg, true);
    }
  }
}

/// Startet den srcp Server
/// # Arguments
/// * config_file_values - Gesamtes Konfigfile
/// * all_info_rx - Alle Channel Receiver der Info Messages aller Server
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
pub fn startup(
  config_file_values: &HashMap<String, HashMap<String, Option<String>>>,
  info_rx: Receiver<SRCPMessage>, all_cmd_tx: &HashMap<usize, Sender<Message>>,
) -> Result<(), String> {
  let port = config_file_values
    .get("srcp")
    .ok_or("Keine [srcp] Abschnitt in Konfiguration")?
    .get("port")
    .ok_or("Keine [srcp] port-Angabe in Konfigfile")?
    .as_ref()
    .ok_or("[srcp] port-Angabe ohne Wert")?
    .parse::<u16>()
    .ok()
    .ok_or("[srcp] port muss eine Zahl sein")?;

  info!("srcp start port={port}");
  //Info Message Dispacther Thread starten
  //Alle Infos Messages der verschiedenen srcp_server_ Instanzen werden von diesem Thread an alle angemeldeten
  //Clients mit Info Mode gesendet
  thread::Builder::new()
    .name("Dispatcher".to_string())
    .spawn(move || {
      dispachter_srcp_info(info_rx);
    })
    .unwrap();

  //Hier geht es weiter mit als Hauptthread der auf eingehende Verbindungen wartet
  //und die Verbindung zwischen den für die Verbindungen gestarteten SRCP Servern und den Bus-Servern herstellt
  srcp_server(port, all_cmd_tx);
}

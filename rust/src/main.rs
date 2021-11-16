use std::{
    io::{Error, Read, Write},
    net::{TcpListener, TcpStream},
    thread::{self, sleep},
    time,
};

pub mod weather;
use crate::weather::{W,Time};
async fn handle_client(mut stream: TcpStream) -> Result<(), Error> {
    let mut buf = [0; 512];
    for _ in 0..1000 {
        let bytes_read = stream.read(&mut buf)?;
        if bytes_read == 0 {
            return Ok(());
        }
        let city = std::str::from_utf8(&buf[..bytes_read])
            .unwrap()
            .to_string();
        let resp = W::get(&city).await.unwrap();
        let time =Time::get().await.unwrap(); 
        let s = format!(
            "{},{},{},{},{} 'C,{}\n",
            time.hour,time.minute,time.seconds,city,resp.main.temp, resp.weather.details.description);
        // sleep(time::Duration::from_secs(50));
        stream.write(s.as_bytes())?;
        println!(
            "{:?} has message: {},respond {:?}",
            stream.peer_addr(),
            std::str::from_utf8(&buf[..bytes_read]).unwrap(),
            s
        );
        // thread::sleep(time::Duration::from_secs(1 as u64));
    }
    Ok(())
}
#[tokio::main]
async fn main() -> std::io::Result<()> {
    let listener = TcpListener::bind("0.0.0.0:4000")?;
    println!("{:?}", listener.local_addr());
    let mut thread_vec: Vec<tokio::task::JoinHandle<()>> = Vec::new();
    for stream in listener.incoming() {
        let stream = stream.expect("failed");
        let handle = tokio::spawn(async {
            handle_client(stream).await.expect("");
        });
        thread_vec.push(handle);
    }
    // for handle in thread_vec {
    //     handle.abort();
    // }

    Ok(())
}

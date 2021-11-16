use exitfailure::ExitFailure;
use reqwest::Url;
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct W {
    coord: Coord,
    pub weather: Weather,
    base: String,
    pub main: Main,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Time {
    year: i32,
    month: i32,
    day: i32,
    pub hour: i32,
    pub minute: i32,
    pub seconds: i32,
}

impl Time {
    pub async fn get() -> Result<Self, ExitFailure> {
        let url = Url::parse("https://www.timeapi.io/api/Time/current/zone?timeZone=PRC")?;

        let time = reqwest::get(url).await?.json::<Time>().await?;

        Ok(time)
    }
}


impl W {
    pub async fn get(city: &String) -> Result<Self, ExitFailure> {
        let url=format!("https://api.openweathermap.org/data/2.5/weather?q={}&units=metric&appid=ee8083c3290e4f3701db939e6893168e",city);
        let url = Url::parse(&*url)?;

        

        let resp = reqwest::get(url).await?.json::<W>().await?;

        Ok(resp)
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct Coord {
    lon: f64,
    lat: f64,
}
#[derive(Serialize, Deserialize, Debug)]
pub struct Weather {
    pub details: Details,
}
#[derive(Serialize, Deserialize, Debug)]
pub struct Details {
    id: i32,
    pub main: String,
    pub description: String,
    icon: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Main {
   pub temp: f64,
    feels_like: f64,
    temp_min: f64,
    temp_max: f64,
    pressure: f64,
    humidity: f64,
}

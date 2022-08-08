let particles = [];
const num = 3000;

let count = 0;
let tChange = 30
let timer = tChange;

var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var temp = 0, press = 0, humid = 0, co = 0, light = 0, dust = 0, wind = 0, rain = 0, moist = 0, uv = 0;

window.addEventListener('load', onload);

function onload(event) {
    setInterval(change, tChange*1000);
    initWebSocket();
}

function onOpen(event) {
    console.log('Connection opened');
    getValues();
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    console.log(event.data);
    var myObj = JSON.parse(event.data);
    temp = myObj["temperature"];
    press = myObj["pressure"];
    humid = myObj["humidity"];
    co = myObj["co"];
    light = myObj["light"];
    wind = myObj["wind"];
    rain = myObj["rain"];
    moist = myObj["moisture"];
    uv = myObj["uv"];
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function rround(v) {
  return parseFloat(v).toFixed(1);
}

function setup() {
  createCanvas(windowWidth, windowHeight);
  for(let i = 0; i < num; i++) {
    particles.push(createVector(random(windowWidth), random(windowHeight)));
  }
  background(0);
}

function fire() {
    let noiseScale = 0.1/2;
    let spd = 2;
    background(0, 9);
    strokeWeight(4);
    stroke(0);
    fill("white");
    textSize(20);
    text('UV: ' + uv + ' index', 20, windowHeight-100);
    text('Temperature: ' + rround(temp) + ' °C', 20, windowHeight-70);
    text('Light: ' + rround(light) + ' %', 20, windowHeight-40);
    s = map(light, 0, 100, 10, 100);
    for(let i = 0; i < num; i++) {
      let p = particles[i];
      g = map (p.y, 0, windowHeight, 0, 255);
      stroke(255, g, 0, s);
      point(p.x, p.y);
      let n = noise(p.y*noiseScale/3, p.x*noiseScale, frameCount*noiseScale);
      let a = TAU*n;
      p.y += cos(a)*spd;
      p.x += sin(a)*spd*0.75*map(temp, 7, 40, 1, 2);
      if(!onScreen(p)) {
        p.x = random(width);
        p.y = random(height);
      }
    }
}

function water() {
    let noiseScale = 0.01/2;
    let spd = 3;
    background(0, 5);
    strokeWeight(4);
    stroke(0);
    fill("white");
    textSize(20);
    text('Humidity: ' + (Math.round(humid*100)/100) + ' %', 20, windowHeight-70);
    text('Rain: ' + Math.round(rain) + ' %', 20, windowHeight-40);
    text('Soil Moisture: ' + Math.round(moist) + ' %', 20, windowHeight-100);
    s = map(humid, 0, 70, 50, 100)
    for(let i = 0; i < ((num-500)*rain/100)+500; i++) {
      let p = particles[i];
      stroke(0,100,255, s);
      point(p.x, p.y);
      let n = noise(p.y*noiseScale, p.x*noiseScale/2, frameCount*noiseScale);
      let a = TAU*n;
      p.y -= cos(a)*spd*2;
      p.x += sin(a)*spd/2;
      if(!onScreen(p)) {
        p.x = random(width);
        p.y = random(height);
      }

    }
}

function air() {
    let noiseScale = 0.001;
    let spd = 3;
    background(0, 20);
    strokeWeight(2);
    stroke(0);
    fill("white");
    textSize(20);
    text('Pressure: ' + rround(press) + ' hPa', 20, windowHeight - 70);
    text('Wind speed: ' + rround(wind) + ' km/h', 20, windowHeight - 40);
    s = map(wind, 0, 2, 1, 2)
    text('CO: ' + rround(co) + ' ppm', 20, windowHeight-100);
    for(let i = 0; i < num; i++) {
      let p = particles[i];
      stroke(0,255,0);
      point(p.x, p.y);
      let n = noise(p.x*noiseScale/2, p.y*noiseScale, frameCount*noiseScale);
      let a = TAU*n;
      p.x += cos(a)*spd*s;
      p.y += sin(a)*spd*random(0, 0.75);
      if(!onScreen(p)) {
        p.x = random(width);
        p.y = random(height);
      }
    }
}

function diff(start, end) {
    start = start.split(":");
    end = end.split(":");
    var startDate = new Date(0, 0, 0, start[0], start[1], 0);
    var endDate = new Date(0, 0, 0, end[0], end[1], 0);
    var diff = endDate.getTime() - startDate.getTime();
    var hours = Math.floor(diff / 1000 / 60 / 60);
    diff -= hours * 1000 * 60 * 60;
    var minutes = Math.floor(diff / 1000 / 60);

    return minutes;
}

function change() {
  count = (count + 1) % 3;
  background(0, 64);
}

function draw() {
  console.log(count)
  if      (count == 0) fire();
  else if (count == 1) air();
  else if (count == 2) water();

}

function onScreen(v) {
  return v.x >= 0 && v.x <= width && v.y >= 0 && v.y <= height;
}

function windowResized() {
   resizeCanvas(windowWidth, windowHeight);
}

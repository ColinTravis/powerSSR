var b = p5.board('COM3', 'arduino');
var led;

function setup() {
  led = b.pin(13, 'LED');
}

function keyPressed() {
  if (keyCode === LEFT_ARROW){
    led.on();
  } else if (keyCode === RIGHT_ARROW) {
    led.off();
  } else if (keyCode === UP_ARROW){
    led.blink();
    console.log('Hello, World!');
  } else if (keyCode === DOWN_ARROW) {
    led.noBlink();
  }
}

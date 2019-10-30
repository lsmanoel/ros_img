/**
 * Milliseconds. 
 * 
 * A millisecond is 1/1000 of a second. 
 * Processing keeps track of the number of milliseconds a program has run.
 * By modifying this number with the modulo(%) operator, 
 * different patterns in time are created.  
 */
 
int i;

void setup() {
  size(640, 480);

}

void draw() { 
  background(255);
  textSize(200);
  fill(0);
  text(i++, 200, 300, 200);
  if(i>999){
    i=0;
  }
  delay(33);
}

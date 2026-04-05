void stateInitialize(){
  delay(3000);

  //Has the leg move in an upward 90 degree angle
  moveToPos(0, Vector3(0,200, 150));
  moveToPos(1, Vector3(0,200, 150));
  moveToPos(2, Vector3(0,200, 150));
  moveToPos(3, Vector3(0,200, 150));
  moveToPos(4, Vector3(0,200, 150));
  moveToPos(5, Vector3(0,200, 150));

  delay(2000);


  //ready position
  moveToPos(0, Vector3(0,145, -172));
  moveToPos(1, Vector3(0,145, -172));
  moveToPos(2, Vector3(0,145, -172));
  moveToPos(3, Vector3(0,145, -172));
  moveToPos(4, Vector3(0,145, -172));
  moveToPos(5, Vector3(0,145, -172));
  //return;

  gethomeFootpos();

  delay(1000);


}
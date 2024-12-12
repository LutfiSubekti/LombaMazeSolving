void sensitivity() {
  // + peka putih
  // - peka hitam
  sensiSensor = 200;

  hitam = 1;
  putih = 0;
}

void jalan_strategi() {
  pilihSensor(front);
  ambilAngkat();
  taruhTurun();
rem(100);
}

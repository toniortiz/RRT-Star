#include "mainwindow.h"
#include "simulator.h"
#include <QApplication>

Simulator *simulator;

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  simulator = new Simulator();

  return a.exec();
}

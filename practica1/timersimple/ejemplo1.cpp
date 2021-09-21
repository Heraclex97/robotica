#include "ejemplo1.h"


ejemplo1::ejemplo1() : Ui_Counter() {
    setupUi(this);
    show();
    connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(pushButton, SIGNAL(clicked()), this, SLOT(doReset()));
    connect(pushButton_2, SIGNAL(clicked()), this, SLOT(doChangePeriod()));
    connect(pushButton_3, SIGNAL(clicked()), this, SLOT(doCurrentPeriod()));
    connect(pushButton_4, SIGNAL(clicked()), this, SLOT(doTotalTime()));
    connect(pushButton_5, SIGNAL(clicked()), this, SLOT(doLapTime()));

    mytimer.connect(std::bind(&ejemplo1::upd, this));
    mytimer.setInit(cont);
    mytimer.start(500);
}

ejemplo1::~ejemplo1() {}

void ejemplo1::doButton() {
    stopped = !stopped;
    if (stopped) {
        mytimer.stop();
    } else {
        mytimer.start(500);
    }
    qDebug() << "click on button";
}

void ejemplo1::doReset() {
    cont = 0;
    mytimer.setInit(cont);
    mytimer.start(500);
    stopped = false;

    qDebug() << "click on reset";
}

void ejemplo1::doCurrentPeriod() {
    qDebug("The current period is: %d", mytimer.getPeriod());
}

void ejemplo1::doTotalTime() {
    qDebug("Total time is: %d", (cont));
}


void ejemplo1::doLapTime() {
    qDebug("Lap: %d", (cont - mytimer.getInit()));
    mytimer.setInit(cont);
}

void ejemplo1::upd() {
    lcdNumber->display(++cont);
    trick++;
}

void ejemplo1::doChangePeriod() {
    mytimer.setPeriod(spinBox->value());
}
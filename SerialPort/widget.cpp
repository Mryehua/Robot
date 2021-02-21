#include "widget.h"
#include "ui_widget.h"
#include <QMessageBox>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    textstate_receive = false;
    textstate_send = false;
    setWindowTitle("RS485通信");
    serialport = new QSerialPort;
    find_port();                    //查找可用串口

    ui->send_button->setEnabled(false);     //设置控件不可用
    ui->close_port->setEnabled(false);

}
Widget::~Widget()
{
    delete ui,serialport;
}

//发送数据
void Widget::on_send_button_clicked()
{
    if(textstate_send == true)  //文版模式
    {
        serialport->write(ui->send_text_window->toPlainText().toLatin1());
    }

    if(textstate_send == false)     //16进制
    {
        QString str = ui->send_text_window->toPlainText();
        int num = str.toInt();
        str = str.setNum(num,16);
        ui->send_text_window->clear();
        ui->send_text_window->append(str);
        serialport->write(ui->send_text_window->toPlainText().toLatin1());
    }


}
//打开串口
void Widget::on_open_port_clicked()
{
   update();
   sleep(100);      //延时100ms
   find_port();     //重新查找com
    //初始化串口
        serialport->setPortName(ui->com->currentText());        //设置串口名
        if(serialport->open(QIODevice::ReadWrite))              //打开串口成功
        {
            serialport->setBaudRate(ui->baud->currentText().toInt());       //设置波特率
            switch(ui->bit->currentIndex())                   //设置数据位数
            {
                case 8:serialport->setDataBits(QSerialPort::Data8);break;
                default: break;
            }
            switch(ui->jiaoyan->currentIndex())                   //设置奇偶校验
            {
                case 0: serialport->setParity(QSerialPort::NoParity);break;
                default: break;
            }
            switch(ui->stopbit->currentIndex())                     //设置停止位
            {
                case 1: serialport->setStopBits(QSerialPort::OneStop);break;
                case 2: serialport->setStopBits(QSerialPort::TwoStop);break;
                default: break;
            }
            serialport->setFlowControl(QSerialPort::NoFlowControl);     //设置流控制
            //连接槽函数
            QObject::connect(serialport, &QSerialPort::readyRead, this, &Widget::Read_Date);
            // 设置控件可否使用
            ui->send_button->setEnabled(true);
            ui->close_port->setEnabled(true);
            ui->open_port->setEnabled(false);
        }
        else    //打开失败提示
        {
            sleep(100);

            QMessageBox::information(this,tr("Erro"),tr("Open the failure"),QMessageBox::Ok);
        }
}
//关闭串口
void Widget::on_close_port_clicked()
{
    serialport->clear();        //清空缓存区
    serialport->close();        //关闭串口

    ui->send_button->setEnabled(false);
    ui->open_port->setEnabled(true);
    ui->close_port->setEnabled(false);
}
//窗口显示串口传来的数据
void Widget::Read_Date()
{
    QByteArray buf;
    buf = serialport->readAll();
    if(!buf.isEmpty())          //将数据显示到文本串口
    {

        if(textstate_receive == true)   //文本模式
        {
            QString str = ui->Receive_text_window->toPlainText();
            str+=tr(buf);
            str += "  ";
            ui->Receive_text_window->clear();
            ui->Receive_text_window->append(str);
        }
        if(textstate_receive == false)   //文本模式
        {
            QString str = ui->Receive_text_window->toPlainText();
            // byteArray 转 16进制
            QByteArray temp = buf.toHex();
            str+=tr(temp);
            str += "  ";
            ui->Receive_text_window->clear();
            ui->Receive_text_window->append(str);
        }
    }
    buf.clear();    //清空缓存区
}


//查找串口
void Widget::find_port()
{
    //查找可用的串口
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);   //设置串口
        if(serial.open(QIODevice::ReadWrite))
        {
            ui->com->addItem(serial.portName());        //显示串口name
            serial.close();
        }
    }
}

//延时函数
void Widget::sleep( int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}


void Widget::on_clear_button_1_clicked()
{
    ui->send_text_window->clear();
}

void Widget::on_clear_button2_clicked()
{
    ui->Receive_text_window->clear();
}

//接收框文本模式转换
void Widget::on_receive_modl_clicked()
{
    if(ui->receive_modl->text() == "文本模式")
    {
        textstate_receive = true;
        ui->receive_modl->setText("hex模式");
    }
    else
    {
        ui->receive_modl->setText("文本模式");
        textstate_receive = false;
    }
}

//发送框文本转换
void Widget::on_send_modl_clicked()
{
    if(ui->send_modl->text() == "文本模式")
    {
        textstate_send = true;
        ui->send_modl->setText("hex模式");
    }
    else
    {
        ui->send_modl->setText("文本模式");
        textstate_send = false;
    }
}


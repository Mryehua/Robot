#include "clientwidget.h"
#include "ui_clientwidget.h"
#include<QHostAddress>

ClientWidget::ClientWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ClientWidget)
{
    ui->setupUi(this);
    this->setWindowTitle("客户端");
    tcpClient=NULL;

    tcpClient =new QTcpSocket(this);

    connect(tcpClient,&QTcpSocket::connected,
            [=]()
    {
        ui->textEditread->setText("成功和服务端连接");
    }
            );
    connect(tcpClient,&QTcpSocket::readyRead,
            [=]()
    {
        //读取对方发送的数据
        QByteArray array = tcpClient->readAll();
        ui->textEditread->append(array);

    }
            );

}

ClientWidget::~ClientWidget()
{
    delete ui;
}

void ClientWidget::on_pushButtonconnect_clicked()
{
    //获取服务器IP与端口
    QString ip =ui->lineEditIP->text();
    qint16 port =ui->lineEditPort->text().toInt();

    //主动连接服务器
    tcpClient->connectToHost(QHostAddress(ip),port);
}

void ClientWidget::on_pushButtonsend_clicked()
{
    //获取编辑区的IP与端口
    QString str = ui->textEditwrite->toPlainText();
    //发送数据
    tcpClient->write(str.toUtf8().data());
}

void ClientWidget::on_pushButtonclose_clicked()
{
    //主动与对方断开连接
    tcpClient->disconnectFromHost();
    tcpClient->close();
}

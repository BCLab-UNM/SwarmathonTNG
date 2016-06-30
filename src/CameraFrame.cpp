#include <CameraFrame.h>

namespace rqt_rover_gui
{

CameraFrame::CameraFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

        frames = 0;
}

void CameraFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);


    image_update_mutex.lock();
    if (!(image.isNull()))
    {
        //painter.drawText(QPoint(50,50), "Image Received From Camera");

        painter.drawImage(contentsRect(), image);

        if(target_c1[0] != -1) 
        {
            QPointF points[4] = {
                QPointF(target_c1[0], target_c1[1]),
                QPointF(target_c2[0], target_c2[1]),
                QPointF(target_c3[0], target_c3[1]),
                QPointF(target_c4[0], target_c4[1])
            };

            painter.drawPolygon(points, 4);

            for(int i = 0; i < 2; i++)
            {
                target_c1[i] = -1;
                target_c2[i] = -1;
                target_c3[i] = -1;
                target_c4[i] = -1;
            }
        }

    }
    else
    {

        painter.drawText(QPoint(50,50), "No Image Received From Camera");
    }
    image_update_mutex.unlock();

    // Track the frames per second for development purposes
    QString frames_per_second;
    frames_per_second = QString::number(frames /(frame_rate_timer.elapsed() / 1000.0), 'f', 0) + " FPS";

    QFontMetrics fm(painter.font());
    painter.drawText(this->width()-fm.width(frames_per_second), fm.height(), frames_per_second);

     frames++;

    if (!(frames % 100)) // time how long it takes to dispay 100 frames
    {
        frame_rate_timer.start();
        frames = 0;
    }

    // end frames per second

}

void CameraFrame::setImage(const QImage& img)
{
    image_update_mutex.lock();
    image = img.copy();
    image_update_mutex.unlock();
    emit delayedUpdate();
}

void CameraFrame::addTarget(double c1[2], double c2[2], double c3[2], double c4[2])
{

    for(int i = 0; i < 2; i++)
    {
        target_c1[i] = c1[i];
        target_c2[i] = c2[i];
        target_c3[i] = c3[i];
        target_c4[i] = c4[i];
    }

}

}


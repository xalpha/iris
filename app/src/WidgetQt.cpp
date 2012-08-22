////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of IrisCC, a C++ UI for camera calibration               //
//                                                                            //
// Copyright (C) 2012 Alexandru Duliu                                         //
//                                                                            //
// IrisCC is free software; you can redistribute it and/or                    //
// modify it under the terms of the GNU  General Public License               //
// as published by the Free Software Foundation; either version 3             //
// of the License, or (at your option) any later version.                     //
//                                                                            //
// IrisCC is distributed in the hope that it will be useful,                  //
// but WITHOUT ANY WARRANTY; without even the implied warranty of             //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU General Public License          //
// along with IrisCC. If not, see <http://www.gnu.org/licenses/>.             //
//                                                                            //
///////////////////////////////////////////////////////////////////////////////

#include <stdexcept>


#include <QApplication>
#include <QGLFormat>
#include <QMouseEvent>

#include <WidgetQt.hpp>


WidgetQt::WidgetQt( QWidget *parent) :
    QGLWidget( parent ),
    m_widget(0)
{
}

WidgetQt::~WidgetQt()
{
    // empty destructor
}


void WidgetQt::initializeGL()
{
    // set format
    QGLFormat format;
    format.setDepth( true );
    format.setAlpha( true );
    format.setSampleBuffers( true );
    format.setSamples( 4 );
    QGLFormat::setDefaultFormat( format );

    // configure stuff
    makeCurrent();
    GLenum err = glewInit();
    if (GLEW_OK != err)
        throw std::runtime_error( "WidgetQt::initializeGL: could not initialize GLEW." );

    // initialize widget
    makeCurrent();
    if( m_widget )
        m_widget->initialize();
}


void WidgetQt::paintGL()
{
    makeCurrent();
    if( m_widget )
    {
        m_widget->draw();
        //update();
    }
}


void WidgetQt::mousePressEvent(QMouseEvent *e)
{
    makeCurrent();
    if( m_widget )
    {
        m_widget->handleMousePress( e->x(), e->y(), convertButton(e->button()) );
        updateGL();
    }
}


void WidgetQt::mouseMoveEvent(QMouseEvent *e)
{
    makeCurrent();
    if( m_widget )
    {
        m_widget->handleMouseMove( e->x(), e->y() );
        updateGL();
    }
}


void WidgetQt::resizeGL ( int width, int height )
{
    makeCurrent();
    if( m_widget )
    {
        m_widget->handleResize( width, height );
        updateGL();
    }
}


const WidgetGL* WidgetQt::widget() const
{
    return m_widget;
}


void WidgetQt::setWidget( WidgetGL* widget )
{
    m_widget = widget;

    // get the color of the system window
    QColor baseCol = QApplication::palette().base().color();

    m_widget->m_baseColor[0] = static_cast<float>(baseCol.redF());
    m_widget->m_baseColor[1] = static_cast<float>(baseCol.greenF());
    m_widget->m_baseColor[2] = static_cast<float>(baseCol.blueF());
}


int WidgetQt::convertButton( Qt::MouseButton qtButton )
{
    switch( qtButton )
    {
        case Qt::NoButton : return WidgetGL::NoButton;
        case Qt::LeftButton : return WidgetGL::LeftButton;
        case Qt::RightButton : return WidgetGL::RightButton;
        case Qt::MidButton : return WidgetGL::MiddleButton;
        case Qt::XButton1 : return WidgetGL::X1Button;
        case Qt::XButton2 : return WidgetGL::X2Button;
        default :
            throw std::runtime_error( "WidgetQt::convertButton: unknown button." );
    }
}


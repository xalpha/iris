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

#pragma once

#include <nox/widget.hpp>

#include <QGLWidget>




class WidgetQt : public QGLWidget
{
    Q_OBJECT

public:
    typedef nox::widget<double> WidgetGL;

public:

    explicit WidgetQt( QWidget *parent = 0);
    virtual ~WidgetQt();

    virtual void initializeGL();
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);

    virtual void resizeGL ( int width, int height );

    const WidgetGL* widget() const;
    void setWidget( WidgetGL* widget );

protected:
    int convertButton( Qt::MouseButton qtButton );

protected:
    WidgetGL* m_widget;

};

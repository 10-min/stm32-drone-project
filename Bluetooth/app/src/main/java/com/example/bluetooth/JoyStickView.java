package com.example.bluetooth;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;

public class JoyStickView extends SurfaceView implements SurfaceHolder.Callback, View.OnTouchListener {

    MainActivity.ConnectedBluetoothThread bluetoothThread;

    public JoyStickView(Context context) {
        super(context);
        getHolder().addCallback(this);
        setOnTouchListener(this);

    }

    public JoyStickView(Context context, AttributeSet attributes, int style) {
        super(context, attributes, style);
        getHolder().addCallback(this);
        setOnTouchListener(this);
    }
    public JoyStickView(Context context, AttributeSet attributes) {
        super(context, attributes);
        getHolder().addCallback(this);
        setOnTouchListener(this);
    }

    float centerX1, centerX2, centerY1, centerY2, baseRadius, hatRadius;
    float newX[] = {0, 0};
    float newY[] = {0, 0};
    byte[] bytes = new byte[] {0,0,0,0,0}; // FB, LR, UD, R, Command

    private void drawJoystick() {
        if (getHolder().getSurface().isValid())
        {
            Canvas myCanvas = this.getHolder().lockCanvas(); //Stuff to draw
            Paint colors = new Paint();
            myCanvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR); // Clear the BG
            colors.setARGB(255, 50, 50, 50);
            myCanvas.drawCircle(centerX1, centerY1, baseRadius, colors);
            myCanvas.drawCircle(centerX2, centerY2, baseRadius, colors);
            colors.setARGB(255, 255,255,255);
            for(int i = 0; i < this.newX.length; i++) {
                myCanvas.drawCircle(this.newX[i], this.newY[i], hatRadius, colors);
            }

            getHolder().unlockCanvasAndPost(myCanvas);
        }
    }

    private void setupDimensions() {
        centerX1 = getWidth() / 3;
        centerY1 = getHeight() / 2;
        centerX2 = (getWidth()*2) / 3;
        centerY2 = getHeight() / 2;
        newX[0] = centerX1;
        newX[1] = centerX2;
        newY[0] = centerY1;
        newY[1] = centerY2 + baseRadius;
        baseRadius = Math.min(getWidth(), getHeight()) / 4;
        hatRadius = Math.min(getWidth(), getHeight()) / 11;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        setupDimensions();
        drawJoystick();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
    int flag[] = {-1, -1};
    @Override
    public boolean onTouch(View v, MotionEvent e) {
        bluetoothThread = ((MainActivity)MainActivity.context_main).threadConnectedBluetooth;
        int pointCount = 0;
        int[] id = {0, 0};

        if (v.equals(this)) {
            if (e.getPointerCount() > 2) {
                pointCount = 2;
            } else
            {
                if (pointCount != e.getPointerCount()) {
                    pointCount = e.getPointerCount();
                    for (int i = 0; i < pointCount; i++) {
                        id[i] = e.getPointerId(i);
                    }
                    if (pointCount == 1 && (flag[0] > -1 && flag[1] > -1)) {
                        if (flag[0] == 0) {
                            if(id[0] == 0) {
                                flag[1] = -1;
                            } else {
                                flag[0] = -1;
                                flag[1] = 0;
                            }
                        } else if(flag[1] == 0) {
                            if(id[0] == 0) {
                                flag[0] = -1;
                            } else {
                                flag[1] = -1;
                                flag[0] = 0;
                            }
                        }
                    }
                }

            }
            if ((e.getAction() & MotionEvent.ACTION_MASK) == MotionEvent.ACTION_DOWN) {
                if (Math.pow(e.getX() - centerX1, 2) + Math.pow(e.getY() - centerY1, 2) <= Math.pow(baseRadius, 2)) {
                    flag[0] = id[0];
                } else if (Math.pow(e.getX() - centerX2, 2) + Math.pow(e.getY() - centerY2, 2) <= Math.pow(baseRadius, 2)) {
                    flag[1] = id[0];
                }
            }
            if ((e.getAction() & MotionEvent.ACTION_MASK) == MotionEvent.ACTION_POINTER_DOWN) {
                if (Math.pow(e.getX(1) - centerX1, 2) + Math.pow(e.getY(1) - centerY1, 2) <= Math.pow(baseRadius, 2)) {
                    if (flag[0] == -1)
                        flag[0] = id[1];
                } else if (Math.pow(e.getX(1) - centerX2, 2) + Math.pow(e.getY(1) - centerY2, 2) <= Math.pow(baseRadius, 2)) {
                    if (flag[1] == -1)
                        flag[1] = id[1];
                }
            }
            if (e.getAction() != MotionEvent.ACTION_UP) {
                float ratio;
                bytes[0] = (byte) ((((centerY1 + baseRadius) - newY[0])/(2*baseRadius)) * 100);
                bytes[1] = (byte) ((((centerX1 + baseRadius) - newX[0])/(2*baseRadius)) * 100);
                bytes[2] = (byte) ((((centerY2 + baseRadius) - newY[1])/(2*baseRadius)) * 100);
                bytes[3] = (byte) ((((centerX2 + baseRadius) - newX[1])/(2*baseRadius)) * 100);
                bytes[4] = 0;
                if(flag[0] > -1) {
                    float displacement1 = (float) Math.sqrt(Math.pow(e.getX(flag[0]) - centerX1, 2) + Math.pow(e.getY(flag[0]) - centerY1, 2));
                    ratio = baseRadius / displacement1;
                    bytes[4] = 'c';

                    if (bluetoothThread != null) {
                        bluetoothThread.write(bytes);
                    }


                    if (displacement1 > baseRadius) {
                        newX[0] = centerX1 + (e.getX(flag[0]) - centerX1) * ratio;
                        newY[0] = centerY1 + (e.getY(flag[0]) - centerY1) * ratio;
                    } else {
                        newX[0] = e.getX(flag[0]);
                        newY[0] = e.getY(flag[0]);
                    }
                } else {
                    newX[0] = centerX1;
                    newY[0] = centerY1;
                }
                if(flag[1] > -1) {
                    float displacement2 = (float) Math.sqrt(Math.pow(e.getX(flag[1]) - centerX2, 2) + Math.pow(e.getY(flag[1]) - centerY2, 2));
                    ratio = baseRadius / displacement2;
                    bytes[4] = 'c';

                    if (bluetoothThread != null) {
                        bluetoothThread.write(bytes);
                    }

                    if (displacement2 > baseRadius) {
                        newX[1] = centerX2 + (e.getX(flag[1]) - centerX2) * ratio;
                        newY[1] = centerY2 + (e.getY(flag[1]) - centerY2) * ratio;
                    } else {
                        newX[1] = e.getX(flag[1]);
                        newY[1] = e.getY(flag[1]);
                    }
                } else {
                    newX[1] = centerX2;
                }
                drawJoystick();

            } else
            {
                flag[0] = -1;
                flag[1] = -1;
                newX[0] = centerX1;
                newX[1] = centerX2;
                newY[0] = centerY1;
                drawJoystick();
            }
        }
        return true;
    }

}

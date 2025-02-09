package com.example.bluetooth;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

public class RCFragment extends Fragment {
    MainActivity.ConnectedBluetoothThread bluetoothThread;
    int f = 0, b = 0, r = 0, l = 0;
    @SuppressLint("ClickableViewAccessibility")
    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_rc, container, false);



        Button btnUp = rootView.findViewById(R.id.btnUp);
        Button btnDown = rootView.findViewById(R.id.btnDown);
        ;
        Button btnRight = rootView.findViewById(R.id.btnRight);
        ;
        Button btnLeft = rootView.findViewById(R.id.btnLeft);
        ;

        bluetoothThread = ((MainActivity) MainActivity.context_main).threadConnectedBluetooth;

        btnUp.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                int status = motionEvent.getAction();
                f = 1;
                if ((r - l) == 0) {
                    go(f - b);
                }

                if (status == MotionEvent.ACTION_UP) {
                    f = 0;
                    go(0);
                }
                return false;
            }
        });
        btnDown.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                int status = motionEvent.getAction();
                b = 1;
                if (r - l == 0) {
                    go(f - b);
                }

                if (status == MotionEvent.ACTION_UP) {
                    b = 0;
                    go(0);
                }
                return false;
            }
        });
        btnRight.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                int status = motionEvent.getAction();
                r = 1;

                turn(r - l);


                if (status == MotionEvent.ACTION_UP) {
                    r = 0;
                    go(0);
                }
                return false;
            }
        });
        btnLeft.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                int status = motionEvent.getAction();
                l = 1;

                turn(r - l);


                if (status == MotionEvent.ACTION_UP) {
                    l = 0;
                    go(0);
                }
                return false;
            }
        });
        return rootView;
    }
    public void go(int d) {
        if (bluetoothThread != null) {
            if (d == 1) {
                bluetoothThread.write("1");
            } else if (d == 0) {
                bluetoothThread.write("0");
            } else if (d == -1) {
                bluetoothThread.write("2");
            }
        }
    }
    public void turn(int d) {
        if (bluetoothThread != null) {
            if (d == 0) {
                go(0);
            } else if (d == 1) {
                bluetoothThread.write("3");
            } else if (d == -1) {
                bluetoothThread.write("4");
            }
        }
    }

}

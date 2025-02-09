package com.example.bluetooth;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

public class DroneFragment extends Fragment {
    MainActivity.ConnectedBluetoothThread bluetoothThread;
    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_drone, container, false);

        bluetoothThread = ((MainActivity) MainActivity.context_main).threadConnectedBluetooth;
        JoyStickView joyStickView = new JoyStickView(requireActivity());
        return rootView;
    }
}

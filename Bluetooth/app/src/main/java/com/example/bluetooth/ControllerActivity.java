package com.example.bluetooth;

import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

public class ControllerActivity extends AppCompatActivity {


    Button btnFragmentDrone;
    Button btnFragmentRC;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_controller);
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });

        this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        DroneFragment fragmentDrone = new DroneFragment();
        RCFragment fragmentRC = new RCFragment();
        

        getSupportFragmentManager().beginTransaction().replace(R.id.controllerFrameLayout, fragmentDrone).commitAllowingStateLoss();

        btnFragmentDrone = findViewById(R.id.btnFragmentDrone);
        btnFragmentRC = findViewById(R.id.btnFragmentRC);

        btnFragmentDrone.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                getSupportFragmentManager().beginTransaction().replace(R.id.controllerFrameLayout, fragmentDrone).commitAllowingStateLoss();
            }
        });
        btnFragmentRC.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                getSupportFragmentManager().beginTransaction().replace(R.id.controllerFrameLayout, fragmentRC).commitAllowingStateLoss();
            }
        });

    }
}
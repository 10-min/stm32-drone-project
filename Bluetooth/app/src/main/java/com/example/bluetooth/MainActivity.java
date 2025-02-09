package com.example.bluetooth;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.activity.result.ActivityResult;
import androidx.activity.result.ActivityResultCallback;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.activity.result.contract.ActivityResultContracts.StartActivityForResult;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Serializable;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    //UI
    Button btnBluetoothOn;
    Button btnBluetoothOff;
    Button btnSendData;
    Button btnController;
    Button btnBluetoothConnect;
    TextView bluetoothStatus;
    TextView receiveData;
    EditText sendData;

    //Bluetooth related
    BluetoothAdapter bluetoothAdapter;
    Set<BluetoothDevice> pairedDevices;
    List<String> listPairedDevices;
    Handler bluetoothHandler;
    ConnectedBluetoothThread threadConnectedBluetooth = null;
    BluetoothDevice bluetoothDevice;
    BluetoothSocket bluetoothSocket;

    //constant and communication UUID
    final static int BT_REQUEST_ENABLE = 1;
    final static int BT_MESSAGE_READ = 2;
    final static int BT_CONNECTING_STATUS = 3;
    final static UUID BT_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    public static Context context_main;






    @SuppressLint("Handlerleak")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });
        context_main = this;
        bluetoothStatus = findViewById(R.id.bluetoothStatus);
        sendData = findViewById(R.id.sendData);
        receiveData = findViewById(R.id.receiveData);
        btnBluetoothOn = findViewById(R.id.btnBluetoothOn);
        btnBluetoothOff = findViewById(R.id.btnBluetoothOff);
        btnBluetoothConnect = findViewById(R.id.btnBluetoothConnect);
        btnSendData = findViewById(R.id.btnSendData);
        btnController = findViewById(R.id.btnController);

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter.isEnabled()) {
            bluetoothStatus.setText("status : ON");
        }
        else
        {
            bluetoothStatus.setText("status : OFF");
        }

        btnBluetoothOn.setOnClickListener(view -> bluetoothOn());

        btnBluetoothOff.setOnClickListener(view -> bluetoothOff());

        btnBluetoothConnect.setOnClickListener(view -> listPairedDevices());

        btnSendData.setOnClickListener(view -> {
            if (threadConnectedBluetooth != null) {
                threadConnectedBluetooth.write(sendData.getText().toString());
                sendData.setText("");
            }
        });

        btnController.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(MainActivity.this, ControllerActivity.class);
                startActivity(intent);
            }
        });



        bluetoothHandler = new Handler(Looper.getMainLooper()) {
            public void handleMessage(android.os.Message msg) {
                if(msg.what == BT_MESSAGE_READ) {
                    String readMessage = null;
                    try {
                        readMessage = new String((byte[]) msg.obj, "UTF-8");
                    } catch (UnsupportedEncodingException e) {
                        e.printStackTrace();
                    }
                    receiveData.setText(readMessage);
                }
            }
        };

    }
    ActivityResultLauncher<Intent> launcher = registerForActivityResult(
            new ActivityResultContracts.StartActivityForResult(),
            new ActivityResultCallback<ActivityResult>() {
                @Override
                public void onActivityResult(ActivityResult result) {
                    if (result.getResultCode() == RESULT_OK) {
                        Toast.makeText(getApplicationContext(), "블루투스 활성화", Toast.LENGTH_SHORT).show();
                        bluetoothStatus.setText("status : ON");
                    } else if (result.getResultCode() == RESULT_CANCELED) {
                        bluetoothStatus.setText("status : OFF");
                    }
                }
            }
    );

    void bluetoothOn() {
        if (bluetoothAdapter == null) {
            Toast.makeText(getApplicationContext(), "블루투스를 지원하지 않는 기기입니다.", Toast.LENGTH_LONG).show();
        }
        else
        {
            if (bluetoothAdapter.isEnabled()) {
                Toast.makeText(getApplicationContext(), "블루투스가 이미 활성화되어 있습니다.", Toast.LENGTH_SHORT).show();
                bluetoothStatus.setText("status : ON");
            }
            else
            {
                Intent intentBluetoothEnable = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                launcher.launch(intentBluetoothEnable);
            }
        }

    }
    @SuppressLint("MissingPermission")
    void bluetoothOff() {
        if (bluetoothAdapter.isEnabled()) {
            bluetoothAdapter.disable();
            Toast.makeText(getApplicationContext(), "비활성화", Toast.LENGTH_SHORT).show();
            bluetoothStatus.setText("status : OFF");
        }
        else {
            Toast.makeText(getApplicationContext(), "블루투스가 이미 비활성화되어 있습니다.", Toast.LENGTH_LONG).show();
            bluetoothStatus.setText("status : OFF");
        }
    }




    @SuppressLint("MissingPermission")
    private void listPairedDevices() {
        if (bluetoothAdapter.isEnabled()) {
            pairedDevices = bluetoothAdapter.getBondedDevices();

            if (pairedDevices.size() > 0) {
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                builder.setTitle("장치 목록");
                listPairedDevices = new ArrayList<String>();

                for (BluetoothDevice devices : pairedDevices) {
                    listPairedDevices.add(devices.getName());
                }
                final CharSequence[] items = listPairedDevices.toArray(new CharSequence[listPairedDevices.size()]);

                builder.setItems(items, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int item) {
                        connectSelectedDevice(items[item].toString());
                    }
                });
                AlertDialog alert = builder.create();
                alert.show();
            } else
            {
                Toast.makeText(getApplicationContext(), "페어링된 장치가 없습니다.", Toast.LENGTH_LONG).show();
            }
        }

    }
    void connectSelectedDevice(String selectedDeviceName) {
        for (BluetoothDevice tempDevice : pairedDevices) {
            if (selectedDeviceName.equals(tempDevice.getName())) {
                bluetoothDevice = tempDevice;
                break;
            }
        }
        try {
            bluetoothSocket = bluetoothDevice.createRfcommSocketToServiceRecord(BT_UUID);
            bluetoothSocket.connect();
            threadConnectedBluetooth = new ConnectedBluetoothThread(bluetoothSocket);
            threadConnectedBluetooth.start();
            bluetoothHandler.obtainMessage(BT_CONNECTING_STATUS, 1, -1).sendToTarget();
        } catch (IOException e) {
            Toast.makeText(getApplicationContext(), "블루투스 연결 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
        }
    }

    public class ConnectedBluetoothThread extends Thread implements Serializable {
        private final BluetoothSocket msocket;
        private final InputStream inputStream;
        private final OutputStream outputStream;

        public ConnectedBluetoothThread(BluetoothSocket socket) {
            msocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "소켓 연결 중 오류가 발생했습니다.", Toast.LENGTH_SHORT).show();
            }

            inputStream = tmpIn;
            outputStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int bytes;

            while (true) {
                try {
                    bytes = inputStream.available();
                    if (bytes != 0) {
                        SystemClock.sleep(100);
                        bytes = inputStream.available();
                        bytes = inputStream.read(buffer, 0, bytes);
                        bluetoothHandler.obtainMessage(BT_MESSAGE_READ, bytes, -1, buffer).sendToTarget();
                    }
                } catch (IOException e) {
                    break;
                }
            }
        }
        public void write(String str) {
            byte[] bytes = str.getBytes();
            try {
                outputStream.write(bytes);
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "데이터 전송 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
            }
        }
        public void write(byte[] bytes) {
            try {
                outputStream.write(bytes);
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "데이터 전송 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
            }
        }
        public void cancel() {
            try {
                msocket.close();
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "소켓 해제 중 오류가 발생했습니다..", Toast.LENGTH_LONG).show();
            }
        }
    }
}
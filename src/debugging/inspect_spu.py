import serial
import datetime
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, axs = plt.subplots(2, 2)
ax_temp = axs[0, 0]
ax_acel = axs[0, 1]
ax_gyro = axs[1, 0]
ax_magn = axs[1, 1]

last_values = []
max_values = 500
total_values = 0
max_temp_seen = 0
min_temp_seen = 999
write_to_disk = False

if write_to_disk:
    log_file = open(f"spu_{str(datetime.datetime.now()).replace(' ', '_').split('.')[0].replace(':', '-')}.output", 'w')
    data_file = open(f"spu_{str(datetime.datetime.now()).replace(' ', '_').split('.')[0].replace(':', '-')}.data", 'w')
arduino = serial.Serial('COM7', 9600, timeout=1)


def try_float(f):
    try:
        return float(f)
    except ValueError:
        return 0.0


def read_from_arduino():
    global last_values, max_values, total_values, max_temp_seen, min_temp_seen
    while True:
        data = arduino.readline()
        if data and b"[DBG]" in data or b"[GBL]" in data:
            output = data.decode('cp437')
            print(output.strip())
            if write_to_disk:
                log_file.write(output)
            if b"[DBG]" in data:
                values = [try_float(v) for v in output.split(" ")[1:][1::2]] + [0]
                if write_to_disk:
                    data_file.write("\t".join([str(v) for v in values[:-1]]) + "\n")
                last_values.append(values)
                if len(last_values) > max_values:
                    last_values = last_values[len(last_values) - max_values:]
                total_values += 1
                max_temp_seen = max(max_temp_seen, values[2])
                min_temp_seen = min(min_temp_seen, values[1])
            if b"[GBL]" in data and b"local error" in data:
                last_values[-1][-1] = 1
            elif b"[GBL]" in data and b"critical error" in data:
                last_values[-1][-1] = 2

threading.Thread(target=read_from_arduino).start()


def animate(_):
    xs = [j for j in range(max(0, total_values - max_values), total_values)]
    average_readings = [k[0] for k in last_values]
    min_readings = [k[1] for k in last_values]
    max_readings = [k[2] for k in last_values]
    diff_readings = [k[3] for k in last_values]
    netAcel_readings = [k[4] for k in last_values]
    acelX_readings = [k[5] for k in last_values]
    acelY_readings = [k[6] for k in last_values]
    acelZ_readings = [k[7] for k in last_values]
    netGyro_readings = [k[8] for k in last_values]
    gyroX_readings = [k[9] for k in last_values]
    gyroY_readings = [k[10] for k in last_values]
    gyroZ_readings = [k[11] for k in last_values]
    netMagn_readings = [k[12] for k in last_values]
    magnX_readings = [k[13] for k in last_values]
    magnY_readings = [k[14] for k in last_values]
    magnZ_readings = [k[15] for k in last_values]

    ax_temp.clear()
    ax_acel.clear()
    ax_gyro.clear()
    ax_magn.clear()

    ax_temp.plot(xs, average_readings, label='Average Temperature', linewidth=2, linestyle='-', color='#ff0000')
    ax_temp.plot(xs, min_readings, label='Min/Max Temperature', linewidth=1, linestyle='--', color='#ff7d7d')
    ax_temp.plot(xs, max_readings, linewidth=1, linestyle='--', color='#ff7d7d')
    ax_temp.plot(xs, [max_temp_seen] * len(xs), label='High/Low Temperature Seen', linewidth=1, linestyle='--', color='#ffc9c9')
    ax_temp.plot(xs, [min_temp_seen] * len(xs), linewidth=1, linestyle='--', color='#ffc9c9')
    ax_acel.plot(xs, netAcel_readings, label='Net', linewidth=1, linestyle='--', color='#707070')
    ax_acel.plot(xs, acelX_readings, label='X', linewidth=1, linestyle='-', color='#ff0000')
    ax_acel.plot(xs, acelY_readings, label='Y', linewidth=1, linestyle='-', color='#00ff00')
    ax_acel.plot(xs, acelZ_readings, label='Z', linewidth=1, linestyle='-', color='#0000ff')
    ax_gyro.plot(xs, netGyro_readings, label='Net', linewidth=1, linestyle='--', color='#707070')
    ax_gyro.plot(xs, gyroX_readings, label='X', linewidth=1, linestyle='-', color='#ff0000')
    ax_gyro.plot(xs, gyroY_readings, label='Y', linewidth=1, linestyle='-', color='#00ff00')
    ax_gyro.plot(xs, gyroZ_readings, label='Z', linewidth=1, linestyle='-', color='#0000ff')
    ax_magn.plot(xs, netMagn_readings, label='Net', linewidth=1, linestyle='--', color='#707070')
    ax_magn.plot(xs, magnX_readings, label='X', linewidth=1, linestyle='-', color='#ff0000')
    ax_magn.plot(xs, magnY_readings, label='Y', linewidth=1, linestyle='-', color='#00ff00')
    ax_magn.plot(xs, magnZ_readings, label='Z', linewidth=1, linestyle='-', color='#0000ff')

    ax_temp.set_ylim(bottom=40, top=120)
    ax_temp.set_title("Temperature Sensors")
    ax_temp.set_ylabel("Temperature (°F)")
    ax_acel.set_ylim(bottom=-20, top=20)
    ax_acel.set_title("Accelerometer")
    ax_acel.set_ylabel("Shock (m/s²)")
    ax_gyro.set_ylim(bottom=-4, top=4)
    ax_gyro.set_title("Gyroscope")
    ax_gyro.set_ylabel("Rotation Delta (radians)")
    ax_magn.set_ylim(bottom=-400, top=400)
    ax_magn.set_title("Magnetometer")
    ax_magn.set_ylabel("Field Strength (uT)")

    for ax in [ax_temp, ax_acel, ax_gyro, ax_magn]:
        if len(last_values) != 0:
            ax.set_xlim(left=max(0, total_values - max_values), right=total_values)
        ax.set_xlabel("Samples")
        ax.legend(loc="upper left")

        local_error = False
        local_start = -1
        for idx, rd in enumerate(last_values):
            if rd[-1] == 1 and not local_error:
                local_error = True
                local_start = idx
            elif rd[-1] == 0 and local_error:
                ax.axvspan(xs[local_start], xs[idx], color='#ffe4b5', alpha=0.5)
                local_error = False
        if local_error:
            ax.axvspan(xs[local_start], xs[-1], color='#ffe4b5', alpha=0.5)


ani = animation.FuncAnimation(fig, animate, interval=250)
plt.tight_layout()
plt.show()

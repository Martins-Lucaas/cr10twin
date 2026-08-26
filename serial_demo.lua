--[[
RS485 Serial Communication Example – Piab Gripper

This example uses a Piab gripper to demonstrate RS485 serial communication.

To communicate through RS485 using raw serial data instead of the standard
Modbus protocol, set the IP address to "127.0.0.1" and the port to 60000.

You can then use TCP commands such as TCPCreate, TCPStart, and TCPWrite
to send RS485 serial data.
]]

-- Create and start the TCP connection
function TCPcreate(ip, port)
    err, socket = TCPCreate(false, ip, port)

    if err ~= 0 then
        print("Creation failed")
        return
    end

    Err = TCPStart(socket, 0)

    if Err ~= 0 then
        TCPDestroy(socket)
        print("Start failed")
    end
end

-- Control the Piab gripper
-- on = 1: Send the ON command
-- Any other value: Send the OFF command
function Main(on)
    if on == 1 then
        TCPWrite(socket, {
            0x01, 0x07, 0x00, 0x01, 0x00,
            0x29, 0x02, 0x00, 0x00, 0x82
        }, 0)
    else
        TCPWrite(socket, {
            0x01, 0x07, 0x00, 0x01, 0x00,
            0x29, 0x02, 0x00, 0x00, 0x97
        }, 0)
    end
end

-- Use the following IP address and port for raw RS485 serial communication
IP = "127.0.0.1"
Port = 60000

-- Create the TCP connection used for RS485 communication
TCPcreate(IP, Port)

-- Send the ON command to the Piab gripper
Main(1)

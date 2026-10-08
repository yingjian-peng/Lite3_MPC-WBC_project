#!/usr/bin/expect
#exit

set ip "192.168.2.1"
set username "ysc"
if {![info exists env(LITE3_PASSWORD)]} {
  puts stderr "Please set LITE3_PASSWORD before running this script."
  exit 1
}
set passwd $env(LITE3_PASSWORD)


set onnx_version "arm"

set script_path [file dirname [info script]]
puts ${script_path}

spawn ssh $username@$ip
expect {
    "(yes/no)" {send "yes\r"; exp_continue }
    "password:" {send "$passwd\r"}
}

expect "$username@*"  {send "mkdir /home/$username/rl_deploy \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/bin \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/lib \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/data \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/lib/onnxruntime \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/lib/onnxruntime/arm \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/lib/onnxruntime/x86 \r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/onnx\r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/onnx/policy\r"}
expect "$username@*"  {send "mkdir /home/$username/rl_deploy/onnx/policy/ppo\r"}
expect "$username@*"  {send "exit\r"}
expect eof 

spawn scp -r ${script_path}/../third_party/onnxruntime/$onnx_version/lib  $username@$ip:/home/$username/rl_deploy/lib/onnxruntime/$onnx_version/lib
expect {
  "密码："
        {
          send "$passwd\n"
        }
   "pass"
        {
          send "$passwd\n"
        }
   "yes/no"
        {
          sleep 5
          send_user "send yes"
          send "yes\n"
        }
   eof
    {
        sleep 5
        send_user "eof\n"
    }
}
set timeout 3000
expect eof

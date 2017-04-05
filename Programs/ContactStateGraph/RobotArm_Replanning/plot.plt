set terminal svg size 1000, 200

set xrange [0:1000]
set yrange [0:1.0]

set output "error_quad_pos.svg"
plot "error.csv" using 1 with lines

set output "error_quad_ori.svg"
plot "error.csv" using 2 with lines

set output "error_kine_pos_t.svg"
plot "error.csv" using 3 with lines

set output "error_kine_pos_r.svg"
plot "error.csv" using 4 with lines

set output "error_kine_vel_t.svg"
plot "error.csv" using 5 with lines

set output "error_kine_vel_r.svg"
plot "error.csv" using 6 with lines

set output "error_force_t.svg"
plot "error.csv" using 7 with lines

set output "error_force_r.svg"
plot "error.csv" using 8 with lines


set yrange [0:5]

set output "target1.svg"
plot "error.csv" using 11 with lines

set output "target2.svg"
plot "error.csv" using 12 with lines


set autoscale y

set output "var_pos_t.svg"
plot "var.csv" using 1 with lines

set output "var_pos_r.svg"
plot "var.csv" using 2 with lines

set output "var_vel_t.svg"
plot "var.csv" using 3 with lines

set output "var_vel_r.svg"
plot "var.csv" using 4 with lines

set output "var_joint_pos.svg"
plot "var.csv" using 5 with lines

set output "var_joint_vel.svg"
plot "var.csv" using 6 with lines

set output "var_force_t.svg"
plot "var.csv" using 7 with lines

set output "var_force_r.svg"
plot "var.csv" using 8 with lines

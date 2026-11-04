if {[catch {connect} err]} { puts "ERR: $err" }
catch {rst -system}
after 1500
targets -set -filter {name =~ "xc7z010*"}
if {[catch {fpga -file D:/ds_analysis/mlx90640_vivado_2/mlx90640_vivado_2.runs/impl_1/design_1.bit} e]} { puts "fpga ERR" } else { puts "PL ok (new bitstream with beep)" }
targets -set -nocase -filter {name =~"APU*"}
targets -set -nocase -filter {name =~"*A9*#0"}
catch {stop}
source D:/ds_analysis/mlx90640_vivado/mlx90640_app/_ide/psinit/ps7_init.tcl
catch {ps7_init}
catch {ps7_post_config}
  # 关键：ps7_init 会把 FCLK0 关掉，必须手动打开，否则 PL 没有时钟（屏幕全黑）
  catch {mwr 0xF8000100 [expr {[mrd -value 0xF8000100] | 0x1}]}
  after 100
targets -set -nocase -filter {name =~"*A9*#0"}
catch {bpremove -all}
rst -processor
after 100
dow D:/ds_analysis/mlx90640_vivado/mlx90640_app/build/mlx90640_app.elf
rwr pc 0x00000000
puts "PC = [rrd pc]"
puts "RUN (OCM)"
catch {con}
after 9000
if {[catch {set p [rrd pc]} e]} { puts "PC n/a" } else { puts "PC = $p" }
if {[catch {set v [mrd -value 0xE000002C]} e]} { puts "UART SR n/a" } else { puts "UART SR = 0x[format %03X $v]" }
if {[catch {set v [mrd -value 0xE0004000]} e]} { puts "I2C0 CR n/a" } else { puts "I2C0 CR = 0x[format %08X $v]" }
puts "DONE"

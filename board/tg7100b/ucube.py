src =Split(''' 
    board.c
''')
component =aos_component('board_tg7100b', src)


global_includes =Split(''' 
    .
''')
for i in global_includes:
    component.add_global_includes(i)

global_macros =Split(''' 
    STDIO_UART=0
    CONFIG_NO_TCPIP
    BOARD_TG7100B
    CONFIG_GPIO_AS_PINRESET
    FLOAT_ABI_HARD
    CONFIG_SOC_SERIES_NRF52X
    CONFIG_BOARD_TG7100B=1
    RHINO_CONFIG_WORKQUEUE=1
''')
for i in global_macros:
    component.add_global_macros(i)

linux_only_targets="netmgrapp nano helloworld bluetooth.blemesh_srv wifihalapp bluetooth.blemesh_cli acapp bluetooth.bleperipheral vflashdemo bluetooth.breezeapp bluetooth.bleadv bluetooth.blemesh bluetooth.ble_bqb helloworld_nocli"

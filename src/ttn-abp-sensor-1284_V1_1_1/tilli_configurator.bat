set /P comportnum="COM Port: "
python3 tilli_configurator_v001.py COM%comportnum% 4800 -pwd -ls
pause
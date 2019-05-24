echo "packing stuff in deepmatching$1.zip"
make clean
make deepmatching-static
make clean
zip /home/wwwlear/people/revaud/data/deepmatching$1.zip *
make

cd ./bin/python_scripts
python3 read_abs.py
result=`python3 check_abs.py`
if [ "$result" = "True" ]
	then
	cd ../
	sudo -S sh pnd_adam_deploy_public.sh
else	
	echo "abs not complete, retry"	
fi

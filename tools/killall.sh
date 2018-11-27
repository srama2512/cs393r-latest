
for tgt in naoqi vision motion restart; do
	pid=$(pgrep ${tgt})
	echo $pid
	if [[ ! -z "$pid" ]]; then
		kill -9 $pid
	fi
done

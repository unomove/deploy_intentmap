export ZMQ=1
ip=$(cat config.json | jq .server)
ip=`sed -e 's/^"//' -e 's/"$//' <<<"$ip"`
cd deploy/bin/
./_intent_map $ip &
./_navd &

cd ../..
./dist_x86_64/client/client

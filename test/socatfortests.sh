#!/bin/bash

pidfile=/tmp/rserialtestsocatpids
PORT1=/tmp/rserial1
PORT2=/tmp/rserial2

exitcode=1

case $1 in
start)
    socat pty,link=$PORT1,raw,echo=0 pty,link=$PORT2,raw,echo=0 > /dev/null 2>&1 &
    socatpid=$!
    if [[ $(ps -p $! -o comm=) == socat ]]; then
        echo $socatpid >> $pidfile
        count=0
        until [[ -e $PORT1 || -e $PORT2 ]]
        do
            echo "waiting for vports $count"
            if (( $count > 100 )); then
                exit 1
            fi
            count=$(($count +1))
            # sleep
        done
        exitcode=0
    fi
    ;;
stop)
    if [[ -e "$pidfile" ]]; then
        while IFS= read -r pid; do
            echo killing $pid
            kill $pid
        done < "$pidfile"
        rm $pidfile
    fi
    exitcode=0
    ;;
esac
exit $exitcode

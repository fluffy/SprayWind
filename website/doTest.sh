#!/bin/sh 

curl --data @testBody.txt http://127.0.0.1:8000/spot.json > foo.html
curl http://127.0.0.1:8000/info > bar.html 
curl http://127.0.0.1:8000 > foobar.html 

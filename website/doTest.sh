#!/bin/sh 

#curl --data @testBody.txt http://spraywind.com/rockblock > foo.html

curl --data @testBody.txt http://127.0.0.1:8000/rockblock > foo.html
curl http://127.0.0.1:8000/info > bar.html 
curl http://127.0.0.1:8000 > foobar.html 

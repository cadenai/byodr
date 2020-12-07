
openssl req -x509 -nodes -days 730 -newkey rsa:2048 -keyout key.pem -out mwlc.pem -config lan.cnf
sudo bash -c 'cat key.pem mwlc.pem >> cert.pem'


Inspect with:
openssl x509 -in cert.pem -text -noout


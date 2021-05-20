# https://github.com/bltavares/docker-images/tree/master/zerotier
# https://hub.docker.com/r/lifeym/zerotier
FROM bltavares/zerotier:1.4.6
#FROM lifeym/zerotier:1.6.5-alpine

CMD ["/usr/sbin/zerotier-one"]
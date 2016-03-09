#!/usr/bin/perl -w

open (IN, $ARGV[0]);
open (OUT,">".$ARGV[0].".out");

while (<IN>) {

    if (/^;/) {
    }
    else{
	printf (OUT "%s", $_);
    }
}

#!/usr/bin/perl -w

use strict;

my $version = 0;

sub version_code($$$)
{
  my $result = shift;
  $result = $result * 256 + shift;
  $result = $result * 256 + shift;
  return $result;
}

#######################################

sub reject_section()
{
  my $level = 0;
  while (<STDIN>) {
    return $_  if ($level == 0 && /^#(?:endif|else)/);
    $level++   if (/^#if/);
    $level--   if (/^#endif/);
  }
}

#######################################

sub accept_section()
{
  my $level = 0;
  my $line;
  while (<STDIN>) {
    if (/^#if\s/ && /\bLINUX_VERSION_CODE\b/ && /\bKERNEL_VERSION\b/) {
      chomp;
      s/^#if\s+//;
      s/\bLINUX_VERSION_CODE\b/$version/;
      s/\bKERNEL_VERSION\b/&version_code/;
      if (eval($_)) {
        $line = &accept_section();
        &reject_section()  if ($line =~ /#else/);
      }
      else {
        $line = &reject_section();
        &accept_section()  if ($line =~ /#else/);
      }
      next;
    }
    return $_  if ($level == 0 && /^#(?:endif|else)/);
    $level++   if (/^#if/);
    $level--   if (/^#endif/);
    print $_;
  }
}

#######################################

if ((scalar(@ARGV) < 1) || ($ARGV[0] eq "-h")) {
  print STDERR "Usage: filter_kernel_version.pl <version> <prog.c >prog-flt.c\n";
  exit 1;
}

my @kvers = split(/\./, $ARGV[0]);
$version = &version_code(@kvers);

accept_section();


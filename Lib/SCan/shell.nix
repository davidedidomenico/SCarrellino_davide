{ pkgs ? import <nixpkgs> { } }:

let

  my-python-packages = ps : with ps; [
        canmatrix # already packaged in nixpkgs
        #( #fetch from pypy
        #  buildPythonPackage rec {
        #      pname = "cantools";
        #      version = "38.0.2";
        #      src = fetchPypi{
        #          inherit pname version;
        #          sha256 = "sha256-k7/m9L1lLzaXY+qRYrAnpi9CSoQA8kI9QRN5GM5oxo4=";
        #      };
        #      doCheck = false;
        #      #propagatedBuildInputs = [
        #      #  # Specify dependencies
        #      #  pkgs.python3Packages.numpy
        #      #];
        #  }
        #)
  ];
in
pkgs.mkShell {
  packages = with pkgs; [
   (python3.withPackages my-python-packages ) 
  ];
}


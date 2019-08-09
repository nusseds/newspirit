with import <nixpkgs> {};
androidenv.buildApp {
  name = "NewSpirit";
  src = ../android;
  release = false;

  # If release is set to true, you need to specify the following parameters
  # keyStore = ./keystore;
  # keyAlias = "myfirstapp";
  # keyStorePassword = "mykeystore";
  # keyAliasPassword = "myfirstapp";

  # Any Android SDK parameters that install all the relevant plugins that a
  # build requires
  platformVersions = [ "28" ];

  # When we include the NDK, then ndk-build is invoked before Ant gets invoked
  includeNDK = true;
}

from conans import ConanFile, CMake

class LibVideiIOConan(ConanFile):
  name = "libvideoio"
  version = "0.1"
  settings = "os", "compiler", "build_type", "arch"
  generators = "cmake"
  options = {"opencv_dir": "ANY"}
  default_options = "opencv_dir=''"
  exports = '*'
  requires = "TCLAP/master@jmmut/testing", \
              "libactive_object/master@amarburg/testing", \
              "g3log/master@amarburg/testing", \
              "liblogger/master@amarburg/testing"

  def config(self):
    self.options["liblogger"].opencv_dir = self.options.opencv_dir
    if self.scope.dev and self.scope.build_tests:
      self.requires( "gtest/1.8.0@lasote/stable" )
      self.options["gtest"].shared = False

  def imports(self):
    self.copy("*.dll", dst="bin", src="bin") # From bin to bin
    self.copy("*.dylib*", dst="bin", src="lib") # From lib to bin

  def build(self):
    cmake = CMake(self.settings)
    if self.options.opencv_dir:
      cmake_opts = "-DOpenCV_DIR=%s" % (self.options.opencv_dir)

    flag_build_tests = "-DBUILD_UNIT_TESTS=1" if self.scope.dev and self.scope.build_tests else ""

    self.run('cmake "%s" %s %s %s' % (self.conanfile_directory, cmake.command_line, cmake_opts, flag_build_tests))
    self.run('cmake --build . %s' % cmake.build_config)
    if self.scope.dev and self.scope.build_tests:
      self.run('cp lib/libvideoio.* bin/')
      # self.run('make unit_test')

  def package(self):
    self.copy("*.h", dst="")
    #if self.options.shared:
    if self.settings.os == "Macos":
        self.copy(pattern="*.dylib", dst="lib", keep_path=False)
    else:
        self.copy(pattern="*.so*", dst="lib", src="lib", keep_path=False)
    #else:
    #    self.copy(pattern="*.a", dst="lib", src="lib", keep_path=False)

  def package_info(self):
      self.cpp_info.libs = ["videoio"]

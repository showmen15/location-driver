// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: location.proto

#ifndef PROTOBUF_location_2eproto__INCLUDED
#define PROTOBUF_location_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2004001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_message_reflection.h>
#include "drivermsg.pb.h"
// @@protoc_insertion_point(includes)

namespace amber {
namespace location_proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_location_2eproto();
void protobuf_AssignDesc_location_2eproto();
void protobuf_ShutdownFile_location_2eproto();

class Location;

// ===================================================================

class Location : public ::google::protobuf::Message {
 public:
  Location();
  virtual ~Location();
  
  Location(const Location& from);
  
  inline Location& operator=(const Location& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const Location& default_instance();
  
  void Swap(Location* other);
  
  // implements Message ----------------------------------------------
  
  Location* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Location& from);
  void MergeFrom(const Location& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // optional double x = 1;
  inline bool has_x() const;
  inline void clear_x();
  static const int kXFieldNumber = 1;
  inline double x() const;
  inline void set_x(double value);
  
  // optional double y = 2;
  inline bool has_y() const;
  inline void clear_y();
  static const int kYFieldNumber = 2;
  inline double y() const;
  inline void set_y(double value);
  
  // @@protoc_insertion_point(class_scope:amber.location_proto.Location)
 private:
  inline void set_has_x();
  inline void clear_has_x();
  inline void set_has_y();
  inline void clear_has_y();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  double x_;
  double y_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];
  
  friend void  protobuf_AddDesc_location_2eproto();
  friend void protobuf_AssignDesc_location_2eproto();
  friend void protobuf_ShutdownFile_location_2eproto();
  
  void InitAsDefaultInstance();
  static Location* default_instance_;
};
// ===================================================================

static const int kGetLocationFieldNumber = 30;
extern ::google::protobuf::internal::ExtensionIdentifier< ::amber::DriverMsg,
    ::google::protobuf::internal::PrimitiveTypeTraits< bool >, 8, false >
  get_location;
static const int kCurrentLocationFieldNumber = 31;
extern ::google::protobuf::internal::ExtensionIdentifier< ::amber::DriverMsg,
    ::google::protobuf::internal::MessageTypeTraits< ::amber::location_proto::Location >, 11, false >
  currentLocation;

// ===================================================================

// Location

// optional double x = 1;
inline bool Location::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Location::set_has_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Location::clear_has_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Location::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline double Location::x() const {
  return x_;
}
inline void Location::set_x(double value) {
  set_has_x();
  x_ = value;
}

// optional double y = 2;
inline bool Location::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Location::set_has_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Location::clear_has_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Location::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline double Location::y() const {
  return y_;
}
inline void Location::set_y(double value) {
  set_has_y();
  y_ = value;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace location_proto
}  // namespace amber

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_location_2eproto__INCLUDED

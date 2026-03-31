#include <gtest/gtest.h>

#include <StringHelper.h>
#include <Identifier.h>
#include <Parameter.h>
#include <SmartPointer.h>

// Phase 4: Verify std::string works throughout Identifier, StringHelper,
// and Parameter after replacing karto::String.

// --- StringHelper::ToString ---

TEST(StringHelper, ToStringBool) {
    EXPECT_EQ(karto::StringHelper::ToString(true), "true");
    EXPECT_EQ(karto::StringHelper::ToString(false), "false");
}

TEST(StringHelper, ToStringIntegers) {
    EXPECT_EQ(karto::StringHelper::ToString(kt_int32s(0)), "0");
    EXPECT_EQ(karto::StringHelper::ToString(kt_int32s(-42)), "-42");
    EXPECT_EQ(karto::StringHelper::ToString(kt_int32u(12345)), "12345");
    EXPECT_EQ(karto::StringHelper::ToString(kt_int64s(-999999999LL)), "-999999999");
}

TEST(StringHelper, ToStringDouble) {
    std::string result = karto::StringHelper::ToString(3.14159, 4);
    EXPECT_EQ(result, "3.1416");
}

TEST(StringHelper, ToStringSizeT) {
    kt_size_t val = 42;
    std::string result = karto::StringHelper::ToString(val);
    EXPECT_EQ(result, "42");
}

// --- StringHelper::FromString ---

TEST(StringHelper, FromStringBool) {
    kt_bool value = false;
    EXPECT_TRUE(karto::StringHelper::FromString("true", value));
    EXPECT_TRUE(value);

    EXPECT_TRUE(karto::StringHelper::FromString("false", value));
    EXPECT_FALSE(value);
}

TEST(StringHelper, FromStringInt32) {
    kt_int32s value = 0;
    EXPECT_TRUE(karto::StringHelper::FromString("-42", value));
    EXPECT_EQ(value, -42);
}

TEST(StringHelper, FromStringDouble) {
    kt_double value = 0.0;
    EXPECT_TRUE(karto::StringHelper::FromString("3.14159", value));
    EXPECT_NEAR(value, 3.14159, 1e-5);
}

// --- StringBuilder ---

TEST(StringBuilder, AppendAndToString) {
    karto::StringBuilder sb;
    sb << "hello" << " " << kt_int32s(42) << " " << 3.14;
    std::string result = sb.ToString();
    EXPECT_NE(result.find("hello"), std::string::npos);
    EXPECT_NE(result.find("42"), std::string::npos);
}

TEST(StringBuilder, AppendSizeT) {
    karto::StringBuilder sb;
    kt_size_t val = 99;
    sb << "count=" << val;
    std::string result = sb.ToString();
    EXPECT_NE(result.find("count=99"), std::string::npos);
}

// --- Identifier (uses std::string internally after Phase 4) ---

TEST(Identifier, DefaultIsEmpty) {
    karto::Identifier id;
    EXPECT_EQ(id.Size(), 0u);
    EXPECT_EQ(id.GetName(), "");
}

TEST(Identifier, SimpleNameNoScope) {
    karto::Identifier id("TestSensor");
    EXPECT_EQ(id.GetName(), "TestSensor");
    EXPECT_EQ(id.GetScope(), "");
    EXPECT_EQ(id.ToString(), "TestSensor");
}

TEST(Identifier, ScopedName) {
    karto::Identifier id("/my_scope/TestSensor");
    EXPECT_EQ(id.GetName(), "TestSensor");
    EXPECT_EQ(id.GetScope(), "my_scope");
    EXPECT_EQ(id.ToString(), "/my_scope/TestSensor");
}

TEST(Identifier, NestedScope) {
    karto::Identifier id("/a/b/Name");
    EXPECT_EQ(id.GetName(), "Name");
    EXPECT_EQ(id.GetScope(), "a/b");
}

TEST(Identifier, LeadingSlashStripped) {
    karto::Identifier id("/Test");
    EXPECT_EQ(id.GetName(), "Test");
    EXPECT_EQ(id.GetScope(), "");
}

TEST(Identifier, EqualityAndCopy) {
    karto::Identifier a("/scope/Name");
    karto::Identifier b("/scope/Name");
    karto::Identifier c("Other");

    EXPECT_TRUE(a == b);
    EXPECT_TRUE(a != c);

    karto::Identifier copy(a);
    EXPECT_TRUE(copy == a);
}

TEST(Identifier, FromStdString) {
    std::string name = "StdStringSensor";
    karto::Identifier id(name);
    EXPECT_EQ(id.GetName(), "StdStringSensor");
}

// --- Parameter (string serialization) ---
// Parameter<T> has a protected destructor (ref-counted via Referenced),
// so we must allocate on the heap. SmartPointer handles cleanup.

TEST(Parameter, Int32GetSetValueAsString) {
    karto::SmartPointer<karto::Parameter<kt_int32s>> param =
        new karto::Parameter<kt_int32s>(nullptr, "TestParam", "Test", "A test param", 42);
    EXPECT_EQ(param->GetValue(), 42);
    EXPECT_EQ(param->GetValueAsString(), "42");

    param->SetValueFromString("-7");
    EXPECT_EQ(param->GetValue(), -7);
}

TEST(Parameter, DoubleGetSetValueAsString) {
    karto::SmartPointer<karto::Parameter<kt_double>> param =
        new karto::Parameter<kt_double>(nullptr, "DoubleParam", "Double", "A double param", 1.5);
    EXPECT_EQ(param->GetValue(), 1.5);

    param->SetValueFromString("2.75");
    EXPECT_NEAR(param->GetValue(), 2.75, 1e-10);
}

TEST(Parameter, BoolGetSetValueAsString) {
    karto::SmartPointer<karto::Parameter<kt_bool>> param =
        new karto::Parameter<kt_bool>(nullptr, "BoolParam", "Bool", "A bool param", true);
    EXPECT_EQ(param->GetValueAsString(), "true");

    param->SetValueFromString("false");
    EXPECT_FALSE(param->GetValue());
}

TEST(Parameter, DefaultValue) {
    karto::SmartPointer<karto::Parameter<kt_int32s>> param =
        new karto::Parameter<kt_int32s>(nullptr, "P", "P", "", 10);
    EXPECT_EQ(param->GetDefaultValue(), 10);

    param->SetValue(99);
    EXPECT_EQ(param->GetValue(), 99);

    param->SetToDefaultValue();
    EXPECT_EQ(param->GetValue(), 10);
}

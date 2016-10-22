// Bring in my package's API, which is what I'm testing
#include <boost/property_tree/ptree.hpp>
#include <canopen_master/objdict.h>

// Bring in gtest
#include <gtest/gtest.h>


template<typename T> canopen::HoldAny parse_int(boost::property_tree::iptree &pt, const std::string &key);

template<typename T> canopen::HoldAny prepare_test(const std::string &str){
    boost::property_tree::iptree pt;
    pt.put("test", str);
    return parse_int<T>(pt, "test");
}

template<typename T> class TestHexTypes :  public ::testing::Test{
public:
    static void test_hex(const T& val, const std::string &str){
        EXPECT_EQ(val, prepare_test<T>(str).template get<T>());
    }
    static void test_hex_node(const T& val, const std::string &str, const uint8_t offset){
        EXPECT_EQ(val, canopen::NodeIdOffset<T>::apply(prepare_test<T>(str),offset));
    }
};

typedef ::testing::Types<uint8_t, uint16_t, uint32_t, uint64_t> PosTypes;

TYPED_TEST_CASE(TestHexTypes, PosTypes);

TYPED_TEST(TestHexTypes, checkZero){
    TestFixture::test_hex(0,"0");
    TestFixture::test_hex(0,"0x0");

    for(uint8_t i = 0; i <=127; ++i){
        TestFixture::test_hex_node(0,"0",i);
        TestFixture::test_hex_node(0,"0x0",i);
        TestFixture::test_hex_node(0+i,"$NODEID+0",i);
        TestFixture::test_hex_node(0+i,"$NODEID+0x0",i);
    }
}
TEST(TestHex, checkCamelCase){
    TestHexTypes<uint16_t>::test_hex(0xABCD, "0xABCD");
    TestHexTypes<uint16_t>::test_hex(0xABCD, "0xabcd");
    TestHexTypes<uint16_t>::test_hex(0xABCD, "0xAbCd");
    TestHexTypes<uint16_t>::test_hex(0xABCD, "0xabCD");
}

TEST(TestHex, checkNodeCamelCase){
    for(uint8_t i = 0; i <=127; ++i){
        TestHexTypes<uint16_t>::test_hex_node(i," $NODEID",i);
        TestHexTypes<uint16_t>::test_hex_node(i," $NODeID",i);
        TestHexTypes<uint16_t>::test_hex_node(i," $NodeId",i);
        TestHexTypes<uint16_t>::test_hex_node(i," $NodeID",i);
        TestHexTypes<uint16_t>::test_hex_node(i," $nodeID",i);
    }
}

TEST(TestHex, checkSpaces){
    TestHexTypes<uint16_t>::test_hex(0xABCD, " 0xABCD ");
    TestHexTypes<uint16_t>::test_hex(0xABCD, "0xABCD ");
    TestHexTypes<uint16_t>::test_hex(0xABCD, " 0xABCD");
}

TEST(TestHex, checkNodeSpaces){
    for(uint8_t i = 0; i <=127; ++i){
        TestHexTypes<uint16_t>::test_hex_node(i," $NODEID ",i);
        TestHexTypes<uint16_t>::test_hex_node(i," $NODEID",i);
        TestHexTypes<uint16_t>::test_hex_node(i,"$NODEID ",i);
        TestHexTypes<uint16_t>::test_hex_node(i+1,"$NODEID + 1",i);
        TestHexTypes<uint16_t>::test_hex_node(i+1,"$NODEID+ 1",i);
        TestHexTypes<uint16_t>::test_hex_node(i+1,"$NODEID+1",i);
        TestHexTypes<uint16_t>::test_hex_node(i+1,"$NODEID +1",i);
        TestHexTypes<uint16_t>::test_hex_node(i+1,"$NODEID +0x1 ",i);
        TestHexTypes<uint16_t>::test_hex_node(i+1,"$NODEID + 0x1",i);
    }
}
TEST(TestHex, checkCommonObjects){
    for(uint8_t i = 0; i <=127; ++i){
        TestHexTypes<uint32_t>::test_hex_node( 0x80+i,"$NODEID+0x80",i); // EMCY

        TestHexTypes<uint32_t>::test_hex_node(0x180+i,"$NODEID+0x180",i); //TPDO1
        TestHexTypes<uint32_t>::test_hex_node(0x200+i,"$NODEID+0x200",i); //RPDO1

        TestHexTypes<uint32_t>::test_hex_node(0x280+i,"$NODEID+0x280",i); //TPDO2
        TestHexTypes<uint32_t>::test_hex_node(0x300+i,"$NODEID+0x300",i); //RPDO2

        TestHexTypes<uint32_t>::test_hex_node(0x380+i,"$NODEID+0x380",i); //TPDO3
        TestHexTypes<uint32_t>::test_hex_node(0x400+i,"$NODEID+0x400",i); //RPDO3

        TestHexTypes<uint32_t>::test_hex_node(0x480+i,"$NODEID+0x480",i); //TPDO4
        TestHexTypes<uint32_t>::test_hex_node(0x500+i,"$NODEID+0x500",i); //RPDO4

        TestHexTypes<uint32_t>::test_hex_node(0x580+i,"$NODEID+0x580",i); // TSDO
        TestHexTypes<uint32_t>::test_hex_node(0x600+i,"$NODEID+0x600",i); // RSDO

        TestHexTypes<uint32_t>::test_hex_node(0x700+i,"$NODEID+0x700",i); //NMT
    }
}

void set_access( canopen::ObjectDict::Entry &entry, std::string access);

void testAccess(bool c, bool r, bool w, const char* variants[]){
  canopen::ObjectDict::Entry entry;
  for(const char ** v = variants; *v; ++v){
      SCOPED_TRACE(*v);
      entry.constant = !c;
      entry.readable = !r;
      entry.writable = !w;

      set_access(entry, *v);

      ASSERT_EQ(c, entry.constant);
      ASSERT_EQ(r, entry.readable);
      ASSERT_EQ(w, entry.writable);
  }
}

TEST(TestAccessString, TestRO)
{
  const char* variants[] = {"ro", "Ro", "rO", "RO", 0};
  testAccess(false, true, false, variants);
}

TEST(TestAccessString, TestWO)
{
  const char* variants[] = {"wo", "Wo", "wO", "WO", 0};
  testAccess(false, false, true, variants);
}

TEST(TestAccessString, TestRW)
{
  const char* variants[] = {
      "rw" , "Rw" , "rW" , "Rw",
      "rwr", "Rwr", "rWr", "Rwr",
      "rwR", "RwR", "rWR", "RwR",
      "rww", "Rww", "rWw", "Rww",
      "rwW", "RwW", "rWW", "RwW",
      0};
  testAccess(false, true, true, variants);
}

TEST(TestAccessString, TestConst)
{
  const char* variants[] = {
      "const" , "Const" , "CONST", 0};
  testAccess(true, true, false, variants);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
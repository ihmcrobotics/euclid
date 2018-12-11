package us.ihmc.euclid.utils;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.utils.NameBasedHashCodeTools.*;

import org.junit.jupiter.api.Test;

public class NameBasedHashCodeToolsTest
{
   private String TEST_STRING = "test string";
   private String GNIRTS_TSET = "gnirts tset";

   private NameBasedHashCodeHolderImpl nullHolder = new NameBasedHashCodeHolderImpl(null);
   private NameBasedHashCodeHolderImpl stringHolder = new NameBasedHashCodeHolderImpl(TEST_STRING);
   private NameBasedHashCodeHolderImpl gnirtsHolder = new NameBasedHashCodeHolderImpl(GNIRTS_TSET);

   private class NameBasedHashCodeHolderImpl implements NameBasedHashCodeHolder
   {
      private long nameBasedHashCode;

      NameBasedHashCodeHolderImpl(String str)
      {
         nameBasedHashCode = str == null ? NULL_HASHCODE : str.hashCode();
      }

      @Override
      public long getNameBasedHashCode()
      {
         return nameBasedHashCode;
      }
   }

   private NameBasedHashCodeHolder[] nameBasedHashCodeHolders = new NameBasedHashCodeHolder[] {nullHolder, stringHolder, gnirtsHolder};

   @Test
   public void testComputeNameBasedHashCode()
   {
      assertEquals(TEST_STRING.hashCode(), NameBasedHashCodeTools.computeStringHashCode(TEST_STRING));
      assertEquals(GNIRTS_TSET.hashCode(), NameBasedHashCodeTools.computeStringHashCode(GNIRTS_TSET));
      assertEquals(NULL_HASHCODE, NameBasedHashCodeTools.computeStringHashCode(null));
   }

   @Test
   public void testCombineNameBasedHashCodes()
   {
      assertEquals(PRIME * TEST_STRING.hashCode() + GNIRTS_TSET.hashCode(), NameBasedHashCodeTools.combineHashCodes(TEST_STRING, GNIRTS_TSET));
      assertEquals(PRIME * NULL_HASHCODE + GNIRTS_TSET.hashCode(), NameBasedHashCodeTools.combineHashCodes(null, GNIRTS_TSET));
      assertEquals(PRIME * GNIRTS_TSET.hashCode() + NULL_HASHCODE, NameBasedHashCodeTools.combineHashCodes(GNIRTS_TSET, nullHolder));
      assertEquals(PRIME * NULL_HASHCODE + TEST_STRING.hashCode(), NameBasedHashCodeTools.combineHashCodes(NULL_HASHCODE, stringHolder));
      assertEquals(PRIME * TEST_STRING.hashCode() + GNIRTS_TSET.hashCode(), NameBasedHashCodeTools.combineHashCodes(stringHolder, gnirtsHolder));
   }

   @Test
   public void testCombineArrayHashCode()
   {
      long hashCode = DEFAULT_HASHCODE;

      for (int i = 0; i < nameBasedHashCodeHolders.length; ++i)
      {
         hashCode = PRIME * hashCode + nameBasedHashCodeHolders[i].getNameBasedHashCode();
      }

      assertEquals(hashCode, NameBasedHashCodeTools.computeArrayHashCode(nameBasedHashCodeHolders));

      assertEquals(hashCode, NameBasedHashCodeTools.computeSubArrayHashCode(nameBasedHashCodeHolders, 0, nameBasedHashCodeHolders.length - 1));
   }
}

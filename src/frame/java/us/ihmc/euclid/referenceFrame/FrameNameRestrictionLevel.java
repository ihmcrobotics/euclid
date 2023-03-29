package us.ihmc.euclid.referenceFrame;

import java.util.HashMap;
import java.util.Map;

/**
 * The constant are declared in order of restriction, i.e. when comparing two levels of restriction,
 * the one with greatest ordinal is the most restrictive.
*/
public enum FrameNameRestrictionLevel
{
   /**
    * No name restriction is imposed.
    */
   NONE,
   /**
    * Two frames cannot have the same {@link ReferenceFrame#getNameId()}.
    */
   NAME_ID,
   /**
    * Two frames cannot have the same {@link ReferenceFrame#getName()}}.
    */
   FRAME_NAME;

   private static final Map<String, FrameNameRestrictionLevel> nameMap = new HashMap<>();

   static
   {
      for (FrameNameRestrictionLevel frameNameRestrictionLevel : values())
      {
         nameMap.put(frameNameRestrictionLevel.name(), frameNameRestrictionLevel);
      }
   }

   public static FrameNameRestrictionLevel loadFromEnvironment(String propertyKey, String environmentVariableName, FrameNameRestrictionLevel defaultValue)
   {
      if (propertyKey != null)
      {
         String stringValue = System.getProperty(propertyKey);
         if (stringValue != null && nameMap.containsKey(stringValue))
            return nameMap.get(stringValue);
      }

      if (environmentVariableName != null)
      {
         String stringValue = System.getenv(environmentVariableName);
         if (stringValue != null && nameMap.containsKey(stringValue))
            return nameMap.get(stringValue);
      }

      return defaultValue;
   }
}

package us.ihmc.euclid.referenceFrame;

import java.util.HashMap;
import java.util.Map;

/**
 * The constant are declared in order of restriction, i.e. when comparing two levels of restriction,
 * the one with greatest ordinal is the most restrictive.
 *
 * For example, suppose there is a reference frame with {@link ReferenceFrame#getNameId()} "World:pelvis".
 * There are options for the FrameNameRestrictionLevel of the frame "World":
 * <li> NONE: there are no restrictions on any other reference frames </li>
 * <li> NAME_ID: it is illegal to instantiate another frame with nameId "World:pelvis" </li>
 * <li> FRAME_NAME: it is illegal to instantiate another descendent frame of World with frameName "pelvis", such as "World:elevator:pelvis" </li>
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

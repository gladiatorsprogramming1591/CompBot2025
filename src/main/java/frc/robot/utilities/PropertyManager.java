package frc.robot.utilities;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;

import edu.wpi.first.wpilibj.Preferences;

/**
 * This class contains basic configurable properties. There is an interface
 * where you can ask for a data type, as well as basic classes that can grab the
 * constants from the network table for faster prototyping.
 *
 * @author PJ
 *
 */
public class PropertyManager {
    public static interface IProperty<TypeT> {
        TypeT getValue();
    }

    public static class ConstantProperty<TypeT> implements IProperty<TypeT> {
        private final TypeT mValue;

        public ConstantProperty(String aKey, TypeT aValue) {
            mValue = aValue;

            Preferences.remove(aKey);
        }

        @Override
        public TypeT getValue() {
            return mValue;
        }
    }

    public static class BaseProperty<TypeT> implements IProperty<TypeT> {
        private final String mKey;
        private final TypeT mDefault;
        private final BiConsumer<String, TypeT> mSetter;
        private final BiFunction<String, TypeT, TypeT> mGetter;

        public BaseProperty(String aKey, TypeT aDefault, BiConsumer<String, TypeT> aSetter,
                            BiFunction<String, TypeT, TypeT> aGetter) {
            mKey = aKey;
            mDefault = aDefault;
            mSetter = aSetter;
            mGetter = aGetter;
        }

        @Override
        public TypeT getValue() {
            if (Preferences.containsKey(mKey)) {
                return mGetter.apply(mKey, mDefault);
            }

            mSetter.accept(mKey, mDefault);
            return mDefault;
        }
    }

    public static class IntProperty extends BaseProperty<Integer> {
        public IntProperty(String aKey, int aDefault) {
            super(aKey, aDefault, Preferences::setInt, Preferences::getInt);
        }
    }

    public static class DoubleProperty extends BaseProperty<Double> {
        public DoubleProperty(String aKey, double aDefault) {
            super(aKey, aDefault, Preferences::setDouble, Preferences::getDouble);
        }
    }

    public static class StringProperty extends BaseProperty<String> {
        public StringProperty(String aKey, String aDefault) {
            super(aKey, aDefault, Preferences::setString, Preferences::getString);
        }
    }

    public static class BooleanProperty extends BaseProperty<Boolean> {
        public BooleanProperty(String aKey, boolean aDefault) {
            super(aKey, aDefault, Preferences::setBoolean, Preferences::getBoolean);
        }
    }
}

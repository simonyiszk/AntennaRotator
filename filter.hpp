#pragma once
#include <cstdint>
#include <array>
#include <concepts>
#include <limits>
#include <numeric>

namespace filter{
    constexpr std::size_t powerOfTwo(std::size_t n) {        
        unsigned int power = 0;
        while (n != 1) {
            n /= 2;
            power++;
        }
        
        return power;
    }
    
    template<typename T>
    concept Integer = std::is_integral_v<T>;
    
    template<std::size_t length, Integer FilteredType, std::size_t input_max = std::numeric_limits<FilteredType>::max(), Integer index_type=uint8_t, Integer sum_type=uint16_t>
    class moving_average{
    private:
        static_assert(length != 0, "The length must not be zero.");
        static_assert((length & (length - 1)) == 0, "The length must be a power of 2.");
        
        std::array<FilteredType, length> buff;
        index_type index;
    public:
        moving_average(){}
        
        void operator<<(const FilteredType in){
            static_assert(length < std::numeric_limits<index_type>::max(), "The length must fit in the index_type");
            
            index++;
            index&=length-1;
            buff[index] = in;
        }
        
        FilteredType get() const{
            static_assert(length*input_max < std::numeric_limits<sum_type>::max(), "The buffer must be able to contain the accumulation");
            
            sum_type s = 0;
            for(const auto i:buff) s+=i;
            return s >> powerOfTwo(length);
        }
    };
}

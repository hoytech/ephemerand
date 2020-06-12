# ephemerand

**ephemerand** is an experimental approach to generating a globally-consistent randomness beacon.

Once every day, the US military takes various measurements of the satellites in the [GPS](https://en.wikipedia.org/wiki/Global_Positioning_System) constellation. These measurements include:

* Satellite ephemeris (information about orbit)
  * Inclination
  * Eccentricity
  * Argument of perigee
* Clock skew
* Ionospheric noise

The `ephemerand` program collects these almanacs from a GPS receiver, hashes them, and then outputs the result.

Any device with a ublox chipset should work. I've tested [this](https://www.amazon.com/Stratux-Vk-162-Remote-Mount-USB/dp/B01EROIUEW/) and [this](https://www.amazon.ca/gp/product/B077G5KBNV).

See the [slides of my ETH UofT hackathon entry](https://hoytech.github.io/presentations/ephemerand/) for more details.

My blockchain security course has some introductory slides about [on-chain randomness generation](https://hoytech.github.io/blockchain-security/lesson4/index.html#/21).


## Building

So far only tested on linux (Ubuntu 18.04).

Init the submodules:

    git submodule update --init

Install dependencies:

    sudo apt-get install build-essential g++ libb2-dev libdocopt-dev

Build (requires C++17 compiler):

    make -j


## Running

Plug in your GPS device, then run:

    sudo ./ephemerand run --verbose

If you see version information from your device then it connected OK:

    # Connection OK. SW: 1.00 (59842) HW: 00070000

If you wait for a bit you'll see it pick up satellites (make sure the sky is in view):

    # Sat #32 (1/31) [2044.147456 -> 24166000920924000054fd00b80da1006e9413009f3b96006a77f500b400e700]
    # Sat #31 (2/31) [2044.147456 -> 9a4b5f00a20b24000046fd00350ca1004dbe3f0061b9ff009a905600fcff0600]
    # Sat #30 (3/31) [2044.147456 -> af1e5e0032ff24000034fd007e0ca100dd604000c7e4840014796700c8fff700]
    # Sat #29 (4/31) [2044.147456 -> 62065d00121c24000065fd001c0ca100b37c9600f2e33a00709efa00c0ff2100]

After all 31 satellites are picked up you'll get the current day's random number:

    rand 0936e456684b04edd449bad5c8aba51e28a5adc98d4f20e560af668644f23665 2044.147456 1552323456

The first parameter after `rand` is the random value. The next is the GPS week and GPS time of week that the almanac is applicable for, separated by `.`. The next is the GPS week/time converted to a unix timestamp. 



## Motivation

In decentralised systems there is often a need for random numbers. Despite the system participants not trusting one another, the following properties are required:

* **Unpredictable**: Nobody should be able to predict the number in advance.
* **Unbiasable**: Nobody should be able to influence the number.

### Real-world values

The whole world is full of random numbers. For example, you could flip a coin a bunch of times. The challenge is proving to everyone else that somebody has honestly recorded them, and distributing them to all the participants. With this in mind, people have suggested using [financial data](https://eprint.iacr.org/2010/361), [national lotteries](https://en.bitcoin.it/wiki/Proof_of_burn), and more.

### Block-hashes

In a blockchain system such as Bitcoin, collections of transactions are grouped into blocks and then hashed. Since these blocks are replicated across the world, they can be used as randomness beacons, however there are [many caveats to be aware of](https://blog.positive.com/predicting-random-numbers-in-ethereum-smart-contracts-e5358c6b8620). For instance, the miner who constructs the block can influence the randomness by choosing to not publish the block if the blockhash is disadvantageous. In non-Proof-of-Work blockchains this problem is even worse, since a miner may be able to rapidly construct many valid blocks and suggest the one that has the most advantageous hash.

### Commit-reveal

Another approach is to have all the participants who have an interest in the random value to submit a hash of their own personally generated random number. After everybody has comitted to their hashes, they then reveal their personally generated random numbers. Every participant verifies everyone else's comitted hashes were computed properly, and then combines the random values into a final random number. Similar to block-hashes, the last participant to reveal can choose to not reveal if the final random number is disadvantageous, so there usually needs to be some sort of punishment for not revealing. There are schemes to combine the randomness from multiple participants (ie [RANDAO](https://github.com/randao/randao)), but these usually influencable by (at least) the last revealer.

### Verifiable Delay Functions

A [Verifiable Delay Functions](https://vdfresearch.org/) is a function that takes a very long time to compute and cannot be parallelized, but can be verified in a much shorter amount of time. The output from some less-than secure randomness source (ie blockhash, or RANDAO) is fed as input into the VDF. Next, people commit to being bound by the final result of the VDF. The deadline for comitting to this output must be before anybody could feasible compute the VDF output. Eventually, the final random value is computed and it can be quickly verified by everyone in much less time than it took to compute originally. Most currently-known VDFs require trusted setups, and tuning the delay parameter requires careful tuning (possibly needing specialized hardware as a benchmark, and adjustment over time).

### NIST randomness beacon

The [NIST randomness beacon](https://www.nist.gov/programs-projects/nist-randomness-beacon) is a system that periodically broadcasts randomly-generated values over the internet, signing with NIST's private key to ensure it hasn't been tampered with. Of course to rely on this you need to trust that NIST is generating the random values honestly.

# bot.py
import os
import random
import json, operator
import datetime
import discord
import asyncio

from dotenv import load_dotenv

from discord.ext import commands, tasks

script_placement = "/home/jeh/"

env_path = script_placement + ".env"

load_dotenv(dotenv_path=env_path)
TOKEN = os.getenv('DISCORD_TOKEN')
GUILD = os.getenv('DISCORD_GUILD')

bot = commands.Bot(command_prefix='$')
server = None
leaderboards_channel = None


@bot.event
async def on_ready():
    global server, leaderboards_channel

    for guild in bot.guilds:
        if guild.name == GUILD:
            server = guild
            break

    ##Hardcoded channel ID fordi Malte er en spasser og skal have alle mulige emojis med
    leaderboards_channel = server.get_channel(759739624448065543)

    current_time = datetime.datetime.now()
    f = open(script_placement+"command_log.txt", "a+")
    f.write("jehBOT was started and joined server: " + server.name + " with ID: " + str(server.id) + " at time: " + str(current_time) + "\n")
    f.close()



@bot.command(name='99', help='Responds with a random quote from Brooklyn 99')
async def nine_nine(ctx):
    brooklyn_99_quotes = [
        'I\'m the human form of the 💯 emoji.',
        'Bingpot!',
        (
            'Cool. Cool cool cool cool cool cool cool, '
            'no doubt no doubt no doubt no doubt.'
        ),
    ]

    response = random.choice(brooklyn_99_quotes)
    await ctx.send(response)

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "99" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='undo', help='undo a previous scoring')
@commands.has_any_role('Skaberen', 'Tech guy', 'Admin')
async def undo(ctx):
    global leaderboards_channel, server
    with open(script_placement+'leaderboards_snapshot.json') as json_file:
        data = json.load(json_file)

    json_file.close()
    with open(script_placement+'leaderboards.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    await leaderboards_channel.send("Scorings have been reverted")

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "undo" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='reset_leaderboards', help='resets the leaderboards')
@commands.has_any_role('Tech guy')
async def reset_leaderboards(ctx):

    global leaderboards_channel, server

    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)

    with open(script_placement+'leaderboards_snapshot.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    for p in data['players']:
        p['score'] = 0
        p['imposter_wins'] = 0
        p['crewmate_wins'] = 0
        p['games'] = 0

    with open(script_placement+'leaderboards.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    await leaderboards_channel.send("Leaderboards have been reset")

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "reset_leaderboards" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='leaderboards', help='Displays the current top 10 on the Among Us leaderboards')
async def leaderboards(ctx):
    global leaderboards_channel, server

    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)
    sorted_obj = dict(data)
    sorted_obj['players'] = sorted(data['players'], key=lambda x: x['score'], reverse=True)
    counter = 1

    embed = discord.Embed(title="*Leaderboard:*", description="\u200b", color=0x28C52B)
    icon = server.icon
    if icon is not None:
        icon = server.icon_url
        embed.set_thumbnail(url=icon)
    for p in sorted_obj['players']:
        embed.add_field(name="#"+str(counter)+"  "+p['name'], value=
        "** Score: ** "+str(p['score'])+
        "\n**Imposter wins:** "+str(p['imposter_wins'])+
        "\n**Crewmate wins:** "+str(p['crewmate_wins'])+
        "\n**Games:** "+str(p['games']), inline=True)
        counter += 1
        if counter > 15:
            break
    json_file.close()

    await leaderboards_channel.send(embed=embed)

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "leaderboards" at time: ' + str(current_time) + "\n")
    f.close()

@bot.command(name='admin_win', help='Award points to imposter1 "[Name]" and imposter2 "[Name]"')
@commands.has_any_role('Skaberen', 'Tech guy', 'Admin')
async def admin_win(ctx, imposter1, imposter2):
    global leaderboards_channel, server
    channel_members = ctx.author.voice.channel.members
    point_award = []
    points = 3
    is_scored = False
    played_game = False
    member_count = 0

    for i in channel_members:
        if i.bot:
            continue
        member_count += 1

    for i in channel_members:
        if i.bot:
            continue

        if i.nick is None:
            if i.name == imposter1 or i.name == imposter2:
                point_award.append(str(i.name))
        else:
            if i.nick == imposter1 or i.nick == imposter2:
                point_award.append(str(i.nick))

    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)

    with open(script_placement+'leaderboards_snapshot.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    for i in point_award:
        for j in data['players']:
            if i == j['name']:
                temp_score = j['score'] + points
                j['score'] = temp_score
                temp_win = j['imposter_wins'] + 1
                j['imposter_wins'] = temp_win
                is_scored = True
                break
        if is_scored == False:
            data['players'].append({
                'name': i,
                'score': points,
                'imposter_wins': 1,
                "crewmate_wins": 0,
                "games": 0
            })
        is_scored = False

    for i in channel_members:
        if i.nick is None:
            for j in data['players']:
                if i.name == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.name,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })
        else:
            for j in data['players']:
                if i.nick == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.nick,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })

        played_game = False

    json_file.close()
    with open(script_placement+'leaderboards.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    message_str = ""
    for i in point_award:
        temp_msg = "{player} has been awarded {point} points(s) and an imposter win\n".format(player=i, point=points)
        message_str += temp_msg
    await leaderboards_channel.send(message_str)

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "admin_win" at time: ' + str(current_time) + "\n")
    f.close()

@bot.command(name='rank', help='Checks the rank and points for player "[Name]"')
async def rank(ctx, player):
    global server, leaderboards_channel

    member_flag = False
    member_id = 0
    counter = 0
    is_player = False

    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)
    sorted_obj = dict(data)
    sorted_obj['players'] = sorted(data['players'], key=lambda x : x['score'], reverse=True)

    for i in server.members:
        if i.bot:
            continue

        if i.nick is None:
            if player == i.name:
                member_flag = True
                member_id = i.id
        else:
            if player == i.nick:
                member_flag = True
                member_id = i.id


    if member_flag:
        ranked_player = server.get_member(member_id)
        pfp = ranked_player.avatar_url
        for p in sorted_obj['players']:
            counter += 1
            if p['name'] == player:
                is_player = True

                ##EMBED
                embed = discord.Embed(title=p['name'], description='**Rank**: #'+str(counter)+"\n**Score:** "+str(p['score']), color=0xEE8700)
                embed.set_thumbnail(url=pfp)
                embed.add_field(name='Games:', value=p['games'], inline=True)
                embed.add_field(name="Imposter wins:", value=p['imposter_wins'], inline=True)
                embed.add_field(name="Crewmate wins:", value=p['crewmate_wins'], inline=True)
                await leaderboards_channel.send(embed=embed)

                break
        if is_player == False:
            await leaderboards_channel.send("Player is not currently ranked")

    if member_flag == False:
        await leaderboards_channel.send("Player is not currently a part of the server")

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "rank" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='admin_lose', help='Award points to crewmates NOT imposter1 "[Name]" and imposter2 "[Name]"')
@commands.has_any_role('Skaberen', 'Tech guy', 'Admin')
async def admin_lose(ctx, imposter1, imposter2):
    global server, leaderboards_channel

    point_award = []
    points = 1
    is_scored = False
    played_game = False

    channel_members = ctx.author.voice.channel.members
    member_count = 0
    for i in channel_members:
        if i.bot:
            continue
        member_count += 1

    for i in channel_members:
        if i.bot:
            continue

        if i.nick is None:
            if i.name != imposter1 and i.name != imposter2:
                point_award.append(str(i.name))
        else:
            if i.nick != imposter1 and i.nick != imposter2:
                point_award.append(str(i.nick))

    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)

    with open(script_placement+'leaderboards_snapshot.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    for i in point_award:
        for j in data['players']:
            if i == j['name']:
                temp_score = j['score'] + points
                j['score'] = temp_score
                temp_win = j['crewmate_wins'] + 1
                j['crewmate_wins'] = temp_win
                is_scored = True
                break
        if is_scored == False:
            data['players'].append({
                'name': i,
                'score': points,
                'imposter_wins': 0,
                "crewmate_wins": 1,
                "games": 0
            })
        is_scored = False

    for i in channel_members:
        if i.nick is None:
            for j in data['players']:
                if i.name == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.name,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })
        else:
            for j in data['players']:
                if i.nick == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.nick,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })
        played_game = False

    json_file.close()
    with open(script_placement+'leaderboards.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    message_str = ""
    for i in point_award:
        temp_msg = "{player} has been awarded {point} points(s) and a crewmate win\n".format(player=i, point=points)
        message_str += temp_msg
    await leaderboards_channel.send(message_str)

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "admin_lose" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='roll_dice', help='Simulates rolling [number] dice with [number] sides')
async def roll(ctx, number_of_dice: int, number_of_sides: int):
    dice = [
        str(random.choice(range(1, number_of_sides + 1)))
        for _ in range(number_of_dice)
    ]
    await ctx.send(', '.join(dice))

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "roll_dice" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='lose', help='Award points to crewmates NOT imposter1 "[Name]" and imposter2 "[Name]"')
async def lose(ctx, imposter1, imposter2):
    global server, leaderboards_channel

    point_award = []
    points = 1
    is_scored = False
    played_game = False

    channel_members = ctx.author.voice.channel.members
    member_count = 0

    for i in channel_members:
        if i.bot:
            continue
        member_count += 1

    for i in channel_members:
        if(i.bot):
            continue

        if i.nick is None:
            if i.name != imposter1 and i.name != imposter2:
                point_award.append(str(i.name))
        else:
            if i.nick != imposter1 and i.nick != imposter2:
                point_award.append(str(i.nick))

    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)

    with open(script_placement+'leaderboards_snapshot.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    for i in point_award:
        for j in data['players']:
            if i == j['name']:
                temp_score = j['score'] + points
                j['score'] = temp_score
                temp_win = j['crewmate_wins'] + 1
                j['crewmate_wins'] = temp_win
                is_scored = True
                break
        if is_scored == False:
            data['players'].append({
                'name': i,
                'score': points,
                'imposter_wins': 0,
                "crewmate_wins": 1,
                "games": 0
            })
        is_scored = False

    for i in channel_members:
        if i.nick is None:
            for j in data['players']:
                if i.name == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.name,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })
        else:
            for j in data['players']:
                if i.nick == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.nick,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })

        played_game = False

    json_file.close()

    message_str = ""
    for i in point_award:
        temp_msg = "{player} will be awarded {point} points(s) and a crewmate win\n".format(player=i, point=points)
        message_str += temp_msg
    msg = await leaderboards_channel.send(message_str + "UPVOTE if this is correct")
    await msg.add_reaction('\N{THUMBS UP SIGN}')

    def check(reaction, user):
        for i in channel_members:
            if(user == i and reaction.count >= 5):
                return user != msg.author and (str(reaction.emoji) == '\N{THUMBS UP SIGN}')

    try:
        reaction, user = await bot.wait_for('reaction_add', timeout=15, check=check)
    except asyncio.TimeoutError:
        await leaderboards_channel.send("Points were NOT scored")
        await discord.Message.delete(msg)
        return
    else:
        # we got a reaction
        with open(script_placement+'leaderboards.json', "w") as outfile:
            json.dump(data, outfile, indent=4)
        outfile.close()
        await leaderboards_channel.send("Points were scored")

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "lose" at time: ' + str(current_time) + "\n")
    f.close()


@bot.command(name='win', help='Award points to imposter1 "[Name]" and imposter2 "[Name]"')
async def win(ctx, imposter1, imposter2):
    global leaderboards_channel, server
    channel_members = ctx.author.voice.channel.members
    point_award = []
    points = 3
    is_scored = False
    played_game = False
    member_count = 0

    for i in channel_members:
        if i.bot:
            continue
        member_count += 1

    for i in channel_members:
        if i.bot:
            continue

        if i.nick is None:
            if i.name == imposter1 or i.name == imposter2:
                point_award.append(str(i.name))
        else:
            if i.nick == imposter1 or i.nick == imposter2:
                point_award.append(str(i.nick))


    with open(script_placement+'leaderboards.json') as json_file:
        data = json.load(json_file)

    with open(script_placement+'leaderboards_snapshot.json', "w") as outfile:
        json.dump(data, outfile, indent=4)
    outfile.close()

    for i in point_award:
        for j in data['players']:
            if i == j['name']:
                temp_score = j['score'] + points
                j['score'] = temp_score
                temp_win = j['imposter_wins'] + 1
                j['imposter_wins'] = temp_win
                is_scored = True
                break
        if is_scored == False:
            data['players'].append({
                'name': i,
                'score': points,
                'imposter_wins': 1,
                "crewmate_wins": 0,
                "games": 0
            })
        is_scored = False

    for i in channel_members:
        if i.nick is None:
            for j in data['players']:
                if i.name == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.name,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })
        else:
            for j in data['players']:
                if i.nick == j['name']:
                    temp_games = j['games'] + 1
                    j['games'] = temp_games
                    played_game = True

            if played_game == False:
                data['players'].append({
                    'name': i.nick,
                    'score': 0,
                    'imposter_wins': 0,
                    "crewmate_wins": 0,
                    "games": 1
                })

        played_game = False

    json_file.close()

    message_str = ""
    for i in point_award:
        temp_msg = "{player} will be awarded {point} points(s) and a imposter win\n".format(player=i, point=points)
        message_str += temp_msg
    msg = await leaderboards_channel.send(message_str + "UPVOTE if this is correct")
    await msg.add_reaction('\N{THUMBS UP SIGN}')

    def check(reaction, user):
        for i in channel_members:
            if(user == i and reaction.count >= 5):
                return user != msg.author and (str(reaction.emoji) == '\N{THUMBS UP SIGN}')

    try:
        reaction, user = await bot.wait_for('reaction_add', timeout=15, check=check)
    except asyncio.TimeoutError:
        await leaderboards_channel.send("Points were NOT scored")
        await discord.Message.delete(msg)
        return
    else:
        # we got a reaction
        with open(script_placement+'leaderboards.json', "w") as outfile:
            json.dump(data, outfile, indent=4)
        outfile.close()
        await leaderboards_channel.send("Points were scored")

    user = ctx.author
    user_name = user.name
    if user.nick is None:
        user_nick = "None"
    else:
        user_nick = user.nick
    current_time = datetime.datetime.now()

    f = open(script_placement+"command_log.txt", "a+")
    f.write("USER " + user_name + " AKA " + user_nick + ' used COMMAND: "win" at time: ' + str(current_time) + "\n")
    f.close()


@bot.event
async def on_command_error(ctx, error):
    if isinstance(error, commands.errors.CheckFailure):
        await ctx.send('You do not have the correct role for this command.')

bot.run(TOKEN)